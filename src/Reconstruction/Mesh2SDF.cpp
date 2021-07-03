#include "Mesh2SDF.h"
#include <set>
namespace dragon
{
namespace reconstruction
{

    void Mesh2SDF(const geometry::TriangleMesh &mesh, CubeHandler &cube_handler, float voxel_resolution)
    {
        geometry::HalfEdge he;
        he.FromTriangleMesh(mesh);
        geometry::TriangleMesh processed_mesh;
        he.ToTriangleMesh(processed_mesh);
        auto &faces = he.faces;
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(processed_mesh.points);
        processed_mesh.ComputeNormals();
        auto &normals = processed_mesh.normals;
        // prepare cubes
        cube_handler.Clear();
        cube_handler.SetVoxelResolution(voxel_resolution);

        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
                std::vector<size_t> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(p, indices, dists, 5);
                double min_distance_abs = 1e7;
                bool sign = 1;
                std::set<int> closest_fids;
                geometry::Point3 projected_p;
                geometry::Vector3 tmp_vec;
                for(size_t id = 0; id != indices.size(); ++id)
                {
                    int closest_vid = indices[id];
                    std::vector<int> connected_fids = he.FindConnectedFaces(closest_vid);
                    closest_fids.insert(connected_fids.begin(), connected_fids.end());
                    if(id == 0)
                    {
                        sign = ((p - processed_mesh.points[closest_vid]).dot(normals[closest_vid]) > 0);
                        min_distance_abs = std::sqrt(dists[id]);
                    }
                }
                // computing for the mininum of distance
                for(auto iter = closest_fids.begin(); iter != closest_fids.end(); ++iter)
                {
                    // std::cout<<i<<" "<<connected_fids[i]<<" "<<connected_fids.size()<<std::endl;
                    geometry::Point3 p1 = faces[*iter]->inc_edge->ori_vertex->coor;
                    geometry::Point3 p2 = faces[*iter]->inc_edge->des_vertex->coor;
                    geometry::Point3 p3 = faces[*iter]->inc_edge->next_edge->des_vertex->coor;

                    geometry::Point3 n1 = normals[faces[*iter]->inc_edge->ori_vertex->id];
                    geometry::Point3 n2 = normals[faces[*iter]->inc_edge->des_vertex->id];
                    geometry::Point3 n3 = normals[faces[*iter]->inc_edge->next_edge->des_vertex->id];

                    bool in_triangle = geometry::CheckPointProjectionInTriangle(p, p1, p2, p3, projected_p);
                    if(in_triangle)
                    {
                        tmp_vec = p - projected_p;  
                        double tmp_dist_abs = tmp_vec.norm();
                        if(tmp_dist_abs < min_distance_abs) 
                        {
                            min_distance_abs = tmp_dist_abs;
                            sign = (((p2 - p1).cross(p3 - p1)).dot(tmp_vec) > 0);
                        }
                    }
                    else
                    {
                        {
                            bool on_line_segment = geometry::CheckPointProjectionOnLineSegment(p, p1, p2, projected_p); 
                            if(on_line_segment)
                            {
                                tmp_vec = p - projected_p; 
                                double tmp_dist_abs = (projected_p - p).norm();
                                
                                if(tmp_dist_abs < min_distance_abs)
                                {
                                    min_distance_abs = tmp_dist_abs;
                                    float beta = (projected_p - p1).norm() / (p2 - p1).norm();
                                    sign = (((1 - beta) * n1 + beta * n2).dot(tmp_vec) > 0);
                                }
                            }
                        }
                        {
                            bool on_line_segment = geometry::CheckPointProjectionOnLineSegment(p, p2, p3, projected_p); 
                            if(on_line_segment)
                            {
                                tmp_vec = p - projected_p; 
                                double tmp_dist_abs = (projected_p - p).norm();
                                if(tmp_dist_abs < min_distance_abs)
                                {
                                    min_distance_abs = tmp_dist_abs;
                                    float beta = (projected_p - p2).norm() / (p3 - p2).norm();
                                    sign = (((1 - beta) * n2 + beta * n3).dot(tmp_vec) > 0);
                                }
                            }
                        }
                        {
                            bool on_line_segment = geometry::CheckPointProjectionOnLineSegment(p, p3, p1, projected_p); 
                            if(on_line_segment)
                            {
                                tmp_vec = p - projected_p; 
                                double tmp_dist_abs = (projected_p - p).norm();
                                if(tmp_dist_abs < min_distance_abs)
                                {
                                    min_distance_abs = tmp_dist_abs;
                                    float beta = (projected_p - p3).norm() / (p1 - p3).norm();
                                    sign = (((1 - beta) * n3 + beta * n1).dot(tmp_vec) > 0);
                                }
                            }
                        }
                    }
                }
                if(!sign) return -min_distance_abs;
                return min_distance_abs;
            };        
        cube_handler.IntegratePoints(processed_mesh.points, get_sdf);
        // for(size_t i = 0; i != mesh.points.size(); ++i)
        // {
        //     AddCube(cube_handler.GetCubeID(mesh.points[i]));
        // }
        // std::cout<<BLUE<<"???"<<RESET<<std::endl; 
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;
    }
    void NormalPCD2Indicator(const geometry::PointCloud &pcd, CubeHandler &cube_handler, float voxel_resolution)
    {
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(pcd.points);
        auto &normals = pcd.normals;
        if(!pcd.HasNormals())
        {
            std::cout<<YELLOW<<"[WARNING]::[NormalPCD2Indicator]::the point cloud doesn't has normals."<<RESET<<std::endl;
            return;
        }
        // prepare cubes
        cube_handler.Clear();
        cube_handler.SetVoxelResolution(voxel_resolution);
        float truncation = cube_handler.GetTruncation();
        float indicator = truncation / 2.5;
        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
                std::vector<size_t> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(p, indices, dists, 1);
                bool sign = 1;
                if(indices.size() > 0 && std::sqrt(dists[0]) < truncation * 0.5)
                {
                    int closest_vid = indices[0];
                    sign = ((p - pcd.points[closest_vid]).dot(normals[closest_vid]) > 0);
                    if(!sign) return -indicator;
                    return indicator;
                }
                return 999;

            };        
        cube_handler.IntegratePoints(pcd.points, get_sdf);
        // for(size_t i = 0; i != mesh.points.size(); ++i)
        // {
        //     AddCube(cube_handler.GetCubeID(mesh.points[i]));
        // }
        // std::cout<<BLUE<<"???"<<RESET<<std::endl; 
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;        
    }
}
}