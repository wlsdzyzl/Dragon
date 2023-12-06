#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/Structure/Graph.h"
using namespace dragon;
void ReadCenterLines(const std::string &path, geometry::Point3List &centers, std::vector<double> &radius)
{
    std::ifstream ifs(path);
    centers.clear();
    radius.clear();
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
    centers.pop_back();
    radius.pop_back();
    std::cout<<"number of centers: "<<centers.size()<<" "<<radius.size()<<std::endl;
}
int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout << "Usage: CenterLine2SDFTest [input_filename] [output_filename] [radius_factor = -1.0] [scaling = -1]"<<std::endl;
        return 0;        
    }

    geometry::TriangleMesh generated_mesh;
    geometry::Point3List centers;
    std::vector<double> radius;
    ReadCenterLines(argv[1], centers, radius);
    std::string output_filename = argv[2];
    double scale = -1;
    double radius_factor = -1.0;
    if(argc > 3)
    radius_factor = std::atof(argv[3]);
    if(argc > 4)
    scale = std::atof(argv[4]);
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
    // compute min spanning trees
    auto msts = graph.GenerateMinSpanningTrees();
    std::cout<<"number of MST: "<<msts.size()<<std::endl;
    if(msts.size() != 2)
    {
        // if the number of msts is 1, we need cut edge.
        // if the number of msts is larger than 2, we need make edges.
    }
    geometry::Graph tree_graph(centers);
    for(size_t i = 0; i != msts.size(); ++i) tree_graph.edges.insert(tree_graph.edges.end(), msts[i].begin(), msts[i].end());
    tree_graph.ComputeNeighbors();
    
    // output tree graph
    // identify end point, and bifurcation point
    tree_graph.colors = geometry::Point3List(tree_graph.vertices.size(), geometry::Point3(1.0, 1.0, 1.0));
    size_t max_id1 = -1, max_id2 = -1;
    double max_z1 = -1e7, max_z2 = -1e7;
    for (size_t i = 0; i != tree_graph.vertices.size(); ++i)
    {
        // count edge
        size_t neight_count = tree_graph.neighbors[i].size();
        if(neight_count == 1)
        {
            tree_graph.colors[i][0] = 0;
            if(tree_graph.vertices[i](0) > max_z2)
            {
                if(tree_graph.vertices[i](0) > max_z1)
                {
                    max_id2 = max_id1;
                    max_id1 = i;
                    max_z2 = max_z1;
                    max_z1 = tree_graph.vertices[i](0);
                }
                else
                {
                    max_id2 = i;
                    max_z2 = tree_graph.vertices[i](0);
                }
            }
        }
        if(neight_count >= 3)
        tree_graph.colors[i][1] = 0;
    }
    // tree_graph.colors[max_id1] = geometry::Point3(1.0, 0.0, 0.0);
    // tree_graph.colors[max_id2] = geometry::Point3(1.0, 0.0, 0.0);
    for(size_t i = 0; i != tree_graph.vertices.size(); ++i)
    {
        tree_graph.vertices[i] /= scale;
    }
    tree_graph.WriteToPLY(output_filename);
    return 0;
}
