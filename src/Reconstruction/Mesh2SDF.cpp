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
    // void CenterLine2SDF(const geometry::Point3List &centers, const std::vector<double> &radius, CubeHandler &cube_handler, float voxel_resolution)
    // {
    //     geometry::Point3List normals;
    //     normals.resize(centers.size());
    //     for(size_t i = 0; i != centers.size(); ++i)
    //     {
    //         if(i == 0) normals[i] = (centers[1] - centers[0]).normalized();
    //         else if(i == centers.size() - 1) normals[i] = (centers[i] - centers[i-1]).normalized();
    //         else normals[i] = ((centers[i+1] - centers[i]).normalized() + (centers[i] - centers[i-1]).normalized()).normalized();
    //         // std::cout<<normals[i]<<std::endl;
    //     }

    //     geometry::KDTree<3> kdtree;
    //     kdtree.BuildTree(centers);
    //     cube_handler.Clear();
    //     cube_handler.SetVoxelResolution(voxel_resolution);

    //     std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
    //             std::vector<size_t> indices;
    //             std::vector<float> dists;
    //             kdtree.KnnSearch(p, indices, dists, 10);
    //             double min_distance = 1e7;
    //             std::set<int> line_seg_ids;
    //             for(size_t i = 0; i != indices.size(); ++i)
    //             {
    //                 if(indices[i] > 0) line_seg_ids.insert(indices[i] - 1);
    //                 if(indices[i] < centers.size() - 1) line_seg_ids.insert(indices[i]);
    //             }

    //             for(auto iter = line_seg_ids.begin(); iter != line_seg_ids.end(); ++iter)
    //             {
    //                 int lid = *iter;
    //                 const geometry::Point3 &p1 = centers[lid];
    //                 const geometry::Point3 &p2 = centers[lid + 1];
    //                 geometry::Point3 &n1 = normals[lid];
    //                 geometry::Point3 &n2 = normals[lid + 1];
    //                 double alpha = -1;
    //                 double a_ = p1.dot(n1) + p2.dot(n2) - p1.dot(n2) - p2.dot(n1);
    //                 double b_ = p1.dot(n2) + p2.dot(n1) - p.dot(n1) + p.dot(n2) - 2 * p2.dot(n2);
    //                 double c_ = p2.dot(n2) - p.dot(n2);
    //                 double tmp = b_ * b_ - 4 * a_ * c_;
    //                 if(tmp >= -1e-7)
    //                 {
    //                     if(tmp < 0) tmp = 0;
    //                     double s1 = (- b_ + std::sqrt(tmp)) / (2 * a_);
    //                     double s2 = (- b_ - std::sqrt(tmp)) / (2 * a_);
    //                     if(s1 >= 0 && s1 <= 1) alpha = s1;
    //                     else alpha = s2;
    //                     if(alpha >= 0 && alpha <= 1)
    //                     {
    //                         double inter_radius = alpha * radius[lid] + (1 - alpha) * radius[lid + 1];
    //                         geometry::Point3 inter_p = alpha * p1 + (1 - alpha) * p2;
    //                         // geometry::Point3 inter_n = alpha * n1 + (1 - alpha) * n2;
    //                         // std::cout<<inter_radius<<std::endl;
    //                         double local_distance = (p - inter_p).norm() - inter_radius;
    //                         // if(local_distance < 0) std::cout<<local_distance<<std::endl;

    //                         if(local_distance < min_distance)
    //                         {
    //                             min_distance = local_distance;
    //                         }
    //                     }
    //                     else
    //                     {
    //                         // std::cout<<"alpha: "<<alpha<<std::endl;
    //                     }
    //                 }
    //                 else 
    //                 {
    //                     std::cout<<tmp<<std::endl;
    //                 }
    //             }
    //             // if(min_distance == 1e7)
    //             // {
    //             //     std::cout<<p<<std::endl;
    //             // }
    //             // std::cout<<min_distance<<std::endl;
    //             return min_distance;

    //         };    
    //     cube_handler.IntegratePoints(centers, get_sdf);
    //     std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;   
    // }
    void CenterLine2SDF(const geometry::Point3List &centers, const std::vector<double> &radius, CubeHandler &cube_handler, float voxel_resolution)
    {
        geometry::Point3List normals;
        normals.resize(centers.size());
        for(size_t i = 0; i != centers.size(); ++i)
        {
            if(i == 0) normals[i] = (centers[1] - centers[0]).normalized();
            else if(i == centers.size() - 1) normals[i] = (centers[i] - centers[i-1]).normalized();
            else normals[i] = ((centers[i+1] - centers[i]).normalized() + (centers[i] - centers[i-1]).normalized()).normalized();
            // std::cout<<normals[i]<<std::endl;
        }
        geometry::Vector3 z_axis(0, 0, 1);
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(centers);
        cube_handler.Clear();
        cube_handler.SetVoxelResolution(voxel_resolution);

        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
                std::vector<size_t> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(p, indices, dists, 5);
                double min_distance = 1e7;
                std::set<int> line_seg_ids;
                for(size_t i = 0; i != indices.size(); ++i)
                {
                    if(indices[i] > 0) line_seg_ids.insert(indices[i] - 1);
                    if(indices[i] < centers.size() - 1) line_seg_ids.insert(indices[i]);
                }

                for(auto iter = line_seg_ids.begin(); iter != line_seg_ids.end(); ++iter)
                {
                    int lid = *iter;
                    const geometry::Point3 &c1 = centers[lid];
                    const geometry::Point3 &c2 = centers[lid + 1];
                    geometry::Point3 &n1 = normals[lid];
                    geometry::Point3 &n2 = normals[lid + 1];
                    double r1 = radius[lid];
                    double r2 = radius[lid + 1];
                    // project p to the centerline       
                    geometry::Point3 projected_p;
                    geometry::CheckPointProjectionOnLineSegment(p, c1, c2, projected_p);    
                    geometry::Vector3 n = ((p - c1).cross(c2 - c1)).normalized();         
                    geometry::Vector3 help_n = (p - projected_p).normalized();
                    // find two intersection points
                    geometry::Point3 inter_p1, inter_p2;
                    {
                        geometry::TransformationMatrix local2global = geometry::TransformationMatrix::Identity();
                        local2global.block<3, 3>(0, 0) = 
                            geometry::RotationMatrixBetweenVectors(z_axis, n1);
                        local2global.block<3, 1>(0, 3) = c1; 
                        
                        geometry::TransformationMatrix global2local = local2global.inverse();
                        // std::cout<<geometry::TransformPoint(global2local, c1)<<std::endl;
                        geometry::Point3 tmp_p = geometry::TransformPoint(global2local, p);
                        geometry::Vector3 tmp_n = geometry::TransformNormal(global2local, n);
                        // two intersection
                        geometry::Point3 solution = geometry::Point3::Zero();
                        solution(0) = r1 / std::sqrt( 1 + (tmp_n(0) * tmp_n(0)) / (tmp_n(1) * tmp_n(1)));
                        solution(1) = - tmp_n(0) * solution(0) / tmp_n(1);
                        geometry::Vector3 tmp_help_n = geometry::TransformNormal(global2local, help_n);
                        if((solution.dot(tmp_help_n) > 0) == (tmp_p.dot(tmp_help_n) > 0))
                            inter_p1 = geometry::TransformPoint(local2global, solution);
                        else
                            inter_p1 = geometry::TransformPoint(local2global, -solution);
                        geometry::Point3 test_p;
                        geometry::CheckPointProjectionInTriangle(tmp_p, solution, geometry::Point3(0, 0, 0), 
                            geometry::TransformPoint(global2local, c2), test_p);

                        // std::cout<<test_p.transpose()<<" "<<tmp_p.transpose()<<std::endl;
                    }

                    {
                        geometry::TransformationMatrix local2global = geometry::TransformationMatrix::Identity();
                        local2global.block<3, 3>(0, 0) = 
                            geometry::RotationMatrixBetweenVectors(z_axis, n2);
                        local2global.block<3, 1>(0, 3) = c2; 
                        geometry::TransformationMatrix global2local = local2global.inverse();
                        geometry::Point3 tmp_p = geometry::TransformPoint(global2local, p);
                        geometry::Vector3 tmp_n = geometry::TransformNormal(global2local, n);

                        // two intersection
                        geometry::Point3 solution = geometry::Point3::Zero();
                        solution(0) = r2  / std::sqrt( 1 + (tmp_n(0) * tmp_n(0)) / (tmp_n(1) * tmp_n(1)) );
                        solution(1) = - tmp_n(0) * solution(0) / tmp_n(1);
                        geometry::Vector3 tmp_help_n = geometry::TransformNormal(global2local, help_n);
                        if((solution.dot(tmp_help_n) > 0) == (tmp_p.dot(tmp_help_n) > 0))
                            inter_p2 = geometry::TransformPoint(local2global, solution);
                        else
                            inter_p2 = geometry::TransformPoint(local2global, -solution);
                    }

                    // we have compute two intersection with surface
                    geometry::Point3 useless_p;
                    bool sign = 1;
                    if(geometry::CheckPointProjectionInTriangle(p, inter_p1, c1, c2, useless_p)
                        ||geometry::CheckPointProjectionInTriangle(p, c2, inter_p2, inter_p1, useless_p))
                    sign = 0;
                    // std::cout<<p.transpose()<<" "<<useless_p.transpose()<<std::endl;
                    double local_distance;
                    if(geometry::CheckPointProjectionOnLineSegment(p, inter_p1, inter_p2, useless_p))
                    {
                        local_distance = (p - useless_p).norm();
                    }
                    else
                    {
                        local_distance = std::min((p - inter_p1).norm(), (p - inter_p2).norm());
                    }
                    if(!sign) local_distance = - local_distance;
                    if(local_distance < min_distance) min_distance = local_distance;
                    // if(local_distance < 0) std::cout<<local_distance<<std::endl;
                }
                return min_distance;

            };    
        cube_handler.IntegratePoints(centers, get_sdf);
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;   
    }
}
}