#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/Structure/Graph.h"
#include "Tool/ColorTable.h"
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <map>
using namespace dragon;
void ReadCenterLines(const std::string &path, geometry::Point3List &centers, std::vector<double> &radius)
{
    std::ifstream ifs(path);
    centers.clear();
    radius.clear();
    // std::string _a;
    // ifs >> _a >> _a >> _a >> _a;
    while(ifs)
    {
        geometry::Point3 center;
        // double useless;
        double r = 0.0;
        ifs >> center[0] >> center[1] >> center[2] >> r;
        // std::cout<<center.transpose()<<std::endl;
        centers.push_back(center);
        // r = r<1.0?1.0:r;
        radius.push_back(r);
    }
    ifs.close();
    centers.pop_back();
    radius.pop_back();
    std::cout<<"number of centers: "<<centers.size()<<" "<<radius.size()<<std::endl;
}
void SaveLabels(const std::string &path, const std::vector<int> & labels)
{
    std::ofstream ofs(path);
    for(size_t i = 0; i != labels.size(); ++i)
    ofs << labels[i] << std::endl;
    ofs.close();
}
void SaveLabelTree(const std::string &path, const std::map<size_t, std::set<size_t>> &child_labels, 
    const std::vector<size_t> &father_labels)
{
    std::ofstream ofs(path);
    // ignore the first labels
    ofs<<"number_of_nodes: "<<father_labels.size() - 1<<std::endl;
    ofs<<"head_nodes: [";
    bool first_node = true;
    for(size_t l = 1; l < father_labels.size(); ++l)
    {
        if(father_labels[l] == 0)
        {
            if(first_node) first_node = false;
            else ofs<<", ";
            ofs<<l;
        }
    }
    ofs<<"]"<< std::endl,

    ofs<<"father_nodes: "<<std::endl;
    for(auto & cl: child_labels)
    {
        if(cl.second.size() > 0)
        {
            ofs<< "\t- ";
            ofs<<cl.first<<": [";
            first_node = true;
            for(auto &l: cl.second)
            {
                if(first_node) first_node = false;
                else ofs<<", ";
                ofs<<l;
            }
            ofs<<"]"<<std::endl;
        }
    }
    ofs.close();
}
void GetLabelTree(std::map<size_t, std::set<size_t>> &child_labels, std::vector<size_t> &father_labels, const std::vector<size_t> &seg_to_label, const std::vector<std::vector<size_t>> & segments,
    const std::vector<size_t> &father_seg_ids, const size_t &label_size)
{
    // save tree of segments.
    child_labels.clear();
    // we ignore the first label  0
    father_labels = std::vector<size_t>(label_size+1, 0);
    for(size_t sid = 0; sid != segments.size(); ++sid)
    {
        size_t father_seg_label = seg_to_label[father_seg_ids[sid]];
        size_t seg_label = seg_to_label[sid];
        if(segments[sid].size() > 0 &&   father_seg_label != seg_label )
        {
            child_labels[father_seg_label].insert(seg_label);
            father_labels[seg_label] = father_seg_label;
        }
    }
    
}
void DeleteSegment(std::vector<std::vector<size_t>> & segments, std::unordered_map<size_t, std::unordered_set<size_t>> &child_seg_ids, 
    std::vector<size_t> &father_seg_ids, std::vector<double> &seg_lengths, size_t did)
{
    // merge current segment to its father
    size_t seg_father = father_seg_ids[did];
    segments[seg_father].insert(segments[seg_father].end(), segments[did].begin(), segments[did].end());
    seg_lengths[seg_father] += seg_lengths[did];
    seg_lengths[did] = 0;
    segments[did].clear();
    // std::cout<<"before childs: "<< child_seg_ids[seg_father].size()<<std::endl;
    // merge current segment's childs to its father
    if(child_seg_ids.find(did) != child_seg_ids.end())
    {
        for(auto &child_id: child_seg_ids[did])
        {
            child_seg_ids[seg_father].insert(child_id);
            father_seg_ids[child_id] = seg_father;
        }
        child_seg_ids.erase(did);
    }
    // delete current segment from its father
    child_seg_ids[seg_father].erase(did);
    // std::cout<<"after childs: "<< child_seg_ids[seg_father].size()<<std::endl;
}
int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout << "Usage: CenterLine2SDFTest [input_pcloud] [input_skeleton] [output_filename] [radius_factor = 1.0] [min_seg_length = 0.15] [min_seg_count = 2] [scaling = -1]"<<std::endl;
        return 0;        
    }

    geometry::TriangleMesh generated_mesh;
    geometry::Point3List centers;
    std::vector<double> radius;
    geometry::PointCloud pcd;
    pcd.LoadFromFile(argv[1]);

    ReadCenterLines(argv[2], centers, radius);

    std::string output_filename = argv[3];
    double scale = -1;
    double radius_factor = 1.0;
    double min_seg_length = 0.15;
    double dead_seg_count = 1;
    size_t min_seg_count = 2;
    if(argc > 4)
    radius_factor = std::atof(argv[4]);
    if(argc > 5)
    min_seg_length = std::atof(argv[5]);
    if(argc > 6)
    min_seg_count = std::atoi(argv[6]);
    if(argc > 7)
    scale = std::atof(argv[7]);

    // scaling
    // maybe we don't need scaling in graph construction
    if(scale <= 0)
    {
        geometry::BoundingBox bb;
        for(size_t i = 0; i != centers.size(); ++i)
        bb.AddPoint(centers[i]);
        scale = 1 / (std::max(bb.y_max - bb.y_min, std::max(bb.x_max - bb.x_min, bb.z_max - bb.z_min)));
        // std::cout<<scale<<" "<<bb.y_max <<" "<< bb.y_min<<" " <<bb.x_max <<" "<< bb.x_min <<" "<<bb.z_max <<" "<< bb.z_min <<std::endl;
        
    }
    for(size_t i = 0; i != centers.size(); ++i)
    {
        centers[i] *= scale;
        radius[i] *= scale;
    }

    // pre-processing: radius clustering
    if(radius_factor > 0)
    {
        geometry::Point3List ccenters;
        std::vector<double> cradius;
        auto clusters = geometry::RadiusClustering(centers, radius, radius_factor);
        for(auto &c: clusters)
        {
            geometry::Point3 tmp_center = geometry::Point3::Zero();
            double tmp_r = 0;
            for(auto &id: c)
            {
                tmp_center += centers[id];
                tmp_r += radius[id];
            }
            ccenters.push_back(tmp_center / c.size());
            cradius.push_back(tmp_r / c.size());
        }
        centers = ccenters;
        radius = cradius;    
    }    
    std::cout<<"(After clustering) number of centers: "<<centers.size()<<" "<<radius.size()<<std::endl;
    // graph construction
    geometry::Graph graph(centers);
    graph.ConstructEdgeAdaptive(5, 12, 60);    
    
    // construct tree
    // compute min spanning trees
    auto msts = graph.GenerateMinSpanningTrees();
    
    if(msts.size() != 2)
    {
        // if the number of msts is 1, we need cut edge.
        // if the number of msts is larger than 2, we need make edges.
        std::cout<<"number of MST is not 2: "<<msts.size()<<std::endl;
    }

    // construct tree graph
    geometry::Graph tree_graph(centers);
    std::vector<std::unordered_set<size_t>> tree_vids(msts.size()); 
    std::vector<size_t> start_vids;
    // get the vertex indices of different trees
    for(size_t i = 0; i != msts.size(); ++i) 
    {
        tree_graph.edges.insert(tree_graph.edges.end(), msts[i].begin(), msts[i].end());
        for (const auto & edge : msts[i])
        {
            tree_vids[i].insert(edge.start);
            tree_vids[i].insert(edge.end);
        }

    }
    tree_graph.ComputeNeighbors();


    std::unordered_set<size_t> key_nodes;
    std::vector<size_t> start_node;
    for(size_t m = 0; m != tree_vids.size(); ++m) 
    {
        size_t max_id = -1;
        double max_z = -1e7;
        for (const auto & i: tree_vids[m])
        {
            // count edge
            size_t neight_count = tree_graph.neighbors[i].size();

            // end point
            if(neight_count == 1)
            {
                if(tree_graph.vertices[i](0) > max_z)
                {
                    max_id = i;
                    max_z = tree_graph.vertices[i](0);
                }

                key_nodes.insert(i);
            }
            // bifurcation point
            if(neight_count >= 3)
            {
                key_nodes.insert(i);
            }   
        }
        start_node.push_back(max_id);
    }

    // travel tree graph
    // std::vector<size_t> travel_id = tree_graph.Travel(max_id1);


    std::vector<int> final_labels(tree_graph.vertices.size(), 0);     
    geometry::Point3List labeled_points;
    std::vector<size_t> labeled_point_ids;
    // for each tree, we do DST and label it based on key nodes
    std::vector<size_t> father_ids(tree_graph.vertices.size(), -1);
    // the corresponding segment id of each vertex
    std::vector<size_t> seg_ids(tree_graph.vertices.size(), -1);
    // the corresponding father segment id of each segment
    std::vector<size_t> father_seg_ids;
    std::vector<std::vector<size_t>> segments;
    std::vector<double> seg_lengths;
    std::unordered_map<size_t, std::unordered_set<size_t>> child_seg_ids;
    size_t current_seg_id = 0;
    std::cout<<"Generating segments ..."<<std::endl;
    for(size_t i = 0; i < msts.size(); ++i)
    {
        
        std::vector<size_t> travel_ids = tree_graph.Travel(start_node[i], father_ids);
        
        std::vector<size_t> current_seg = {travel_ids[0]};

        seg_ids[travel_ids[0]] = current_seg_id;
        
        for(size_t j = 1; j < travel_ids.size(); ++j)
        {
            current_seg.push_back(travel_ids[j]);
            seg_ids[travel_ids[j]] = current_seg_id;
            if(key_nodes.find(travel_ids[j]) != key_nodes.end()) 
            {
                // if(current_seg.size() < 2) continue;
                double seg_length = 0;
                for(size_t l = 1; l != current_seg.size(); ++l)
                seg_length += geometry::Distance(tree_graph.vertices[current_seg[l]], tree_graph.vertices[current_seg[l-1]]);
                
                //
                segments.push_back(current_seg);
                size_t seg_father_id = seg_ids[ father_ids[current_seg[0]]];
                father_seg_ids.push_back(seg_father_id);
                seg_lengths.push_back(seg_length);
                child_seg_ids[seg_father_id].insert(current_seg_id);
                current_seg_id += 1;
                current_seg.clear();

            }
        }

    }
    // clean segment with one points
    std::cout<<"clean dead segments ..."<<std::endl;
    for(size_t i = 0; i != segments.size(); ++i)
    {
        if(father_seg_ids[i] != i && segments[i].size() <= dead_seg_count)
        {
            // std::cout<<"clean dead segment "<<i<<std::endl;
            DeleteSegment(segments, child_seg_ids, father_seg_ids, seg_lengths, i);
        }
    }
    // clean segment whose father has only one child
    std::cout<<"clean segments whose father has only one child ..."<<std::endl;
    for(size_t i = 0; i != segments.size(); ++i)
    {
        if(seg_lengths[i] <= 0) continue;
        size_t seg_father = father_seg_ids[i];
        // std::cout<<child_seg_ids[seg_father].size()<<std::endl;
        if(seg_father != i &&  child_seg_ids.find(seg_father) != child_seg_ids.end() && child_seg_ids[seg_father].size() == 1)
        {
            DeleteSegment(segments, child_seg_ids, father_seg_ids, seg_lengths, i);
        }
    }

    // label segments who is qualified
    int label_ptr = 1;
    std::vector<size_t> unlabeled_seg_ids;
    std::cout<<"label qualified segments ..."<<std::endl;
    std::vector<size_t> seg_to_label(segments.size());
    for(size_t i = 0; i != segments.size(); ++i)
    {
        if(segments[i].size() >= min_seg_count && seg_lengths[i] >= min_seg_length)
        {
            seg_to_label[i] = label_ptr;
            for(auto &s: segments[i])
            {
                final_labels[s] = label_ptr;
                labeled_points.push_back(tree_graph.vertices[s]);
                labeled_point_ids.push_back(s);
            }
            label_ptr += 1;
        }
        else if(segments[i].size() > 0)
        {
            unlabeled_seg_ids.push_back(i);
        }
    }

    geometry::KDTree<3> kdtree;
    std::vector<size_t> indices;
    std::vector<float> dists;

    kdtree.BuildTree(labeled_points);
    for(size_t i = 0; i != unlabeled_seg_ids.size(); ++i)
    {
        auto &useg = segments[unlabeled_seg_ids[i]];

        if(father_ids[useg[0]] != useg[0] )
        {
            for(auto &uid: useg)
            final_labels[uid] = final_labels[father_ids[useg[0]]];
            
            seg_to_label[unlabeled_seg_ids[i]] = final_labels[father_ids[useg[0]]];
            
        }
        else
        {
            kdtree.KnnSearch(tree_graph.vertices[useg.back()], indices, dists, 1);
            for(auto &uid: useg)
            final_labels[uid] = final_labels[labeled_point_ids[indices[0]]];

            seg_to_label[unlabeled_seg_ids[i]] = final_labels[labeled_point_ids[indices[0]]];
        }
    }
    
    std::cout<<"Number of labels: "<<label_ptr - 1<<std::endl;

    tree_graph.colors = geometry::Point3List(tree_graph.vertices.size(), geometry::Point3(1.0, 1.0, 1.0));
    for(size_t i = 0; i != tree_graph.vertices.size(); ++i)
    {
        tree_graph.colors[i] = tool::COLOR_TABLE[final_labels[i]];
        tree_graph.vertices[i] /= scale;
    }

    std::cout<<"Colorizing graph ... "<<std::endl;
    tree_graph.WriteToPLY(output_filename);
    geometry::Graph key_graph = tree_graph.GenerateKeyGraph(std::vector<size_t>(key_nodes.begin(), key_nodes.end()));
    key_graph.WriteToPLY(output_filename+std::string(".key.ply"));

    // find label of point cloud based KNN search
    kdtree.BuildTree(tree_graph.vertices);

    std::vector<int> pcd_labels(pcd.GetSize());
    pcd.colors = geometry::Point3List(pcd.points.size(), geometry::Point3(1.0, 1.0, 1.0));
    for(size_t i = 0; i != pcd.points.size(); ++i)
    {
        kdtree.KnnSearch(pcd.points[i], indices, dists, 1);
        pcd_labels[i] = final_labels[indices[0]];
        pcd.colors[i] = tool::COLOR_TABLE[pcd_labels[i]-1];
    }
    // save colorized point cloud
    pcd.WriteToPLY(output_filename+std::string(".seg.ply"));
    // save labels
    SaveLabels(output_filename+std::string(".seg"), pcd_labels);

    // compute label tree
    std::map<size_t, std::set<size_t>> child_labels;
    std::vector<size_t> father_labels;
    GetLabelTree(child_labels, father_labels, seg_to_label, segments, father_seg_ids, label_ptr - 1);
    SaveLabelTree(output_filename+std::string(".tree.yaml"), child_labels, father_labels);
    return 0;
}
