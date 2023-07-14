#include "Mesh2SDF.h"
#include "Geometry/Structure/Graph.h"
#include <set>
#include <unordered_map>
#include <omp.h>
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
                double final_distance_abs = 1e7;
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
                        final_distance_abs = std::sqrt(dists[id]);
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
                        if(tmp_dist_abs < final_distance_abs) 
                        {
                            final_distance_abs = tmp_dist_abs;
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
                                
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
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
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
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
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
                                    float beta = (projected_p - p3).norm() / (p1 - p3).norm();
                                    sign = (((1 - beta) * n3 + beta * n1).dot(tmp_vec) > 0);
                                }
                            }
                        }
                    }
                }
                if(!sign) return -final_distance_abs;
                return final_distance_abs;
            };        
        cube_handler.IntegratePoints(processed_mesh.points, get_sdf);
        // for(size_t i = 0; i != mesh.points.size(); ++i)
        // {
        //     AddCube(cube_handler.GetCubeID(mesh.points[i]));
        // }
        // std::cout<<BLUE<<"???"<<RESET<<std::endl; 
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;
    }
    std::vector<double> Mesh2SDF(const geometry::TriangleMesh &mesh, const geometry::Point3List & points)
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
        std::vector<double> sdfs;
        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
                std::vector<size_t> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(p, indices, dists, 5);
                double final_distance_abs = 1e7;
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
                        final_distance_abs = std::sqrt(dists[id]);
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
                        if(tmp_dist_abs < final_distance_abs) 
                        {
                            final_distance_abs = tmp_dist_abs;
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
                                
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
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
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
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
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
                                    float beta = (projected_p - p3).norm() / (p1 - p3).norm();
                                    sign = (((1 - beta) * n3 + beta * n1).dot(tmp_vec) > 0);
                                }
                            }
                        }
                    }
                }
                if(!sign) return -final_distance_abs;
                return final_distance_abs;
            };        
        for(size_t i = 0; i != points.size(); ++i)
        {
            sdfs.push_back( get_sdf(points[i]));
        }
        // for(size_t i = 0; i != mesh.points.size(); ++i)
        // {
        //     AddCube(cube_handler.GetCubeID(mesh.points[i]));
        // }
        // std::cout<<BLUE<<"???"<<RESET<<std::endl; 
        return sdfs;
    }
    std::vector<double> Mesh2SDF(const geometry::TriangleMesh &mesh, const geometry::Point3i & grid_num, geometry::Point3 origin, float voxel_resolution)
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
        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
                std::vector<size_t> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(p, indices, dists, 5);
                double final_distance_abs = 1e7;
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
                        final_distance_abs = std::sqrt(dists[id]);
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
                        if(tmp_dist_abs < final_distance_abs) 
                        {
                            final_distance_abs = tmp_dist_abs;
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
                                
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
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
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
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
                                if(tmp_dist_abs < final_distance_abs)
                                {
                                    final_distance_abs = tmp_dist_abs;
                                    float beta = (projected_p - p3).norm() / (p1 - p3).norm();
                                    sign = (((1 - beta) * n3 + beta * n1).dot(tmp_vec) > 0);
                                }
                            }
                        }
                    }
                }
                if(!sign) return -final_distance_abs;
                return final_distance_abs;
        };
        std::vector<double> sdfs(size_t(grid_num(0)) * grid_num(1) * grid_num(2));
#pragma omp parallel for
        for(size_t x = 0; x != size_t(grid_num(0)); ++x)
            for(size_t y = 0; y != size_t(grid_num(1)); ++y)
                for(size_t z = 0; z != size_t(grid_num(2)); ++z)
                {
                    geometry::Point3 tmp_p = origin + geometry::Point3(x * voxel_resolution, y * voxel_resolution, z * voxel_resolution);
                    sdfs[x * grid_num(1) * grid_num(2) + y * grid_num(2) + z] = get_sdf(tmp_p);
                }
        return sdfs;
    }
    std::vector<double> Mesh2Indicator(const geometry::TriangleMesh &mesh, const geometry::Point3i & grid_num, geometry::Point3 origin, float voxel_resolution, double truncation, bool coarse)
    {
        if(!mesh.HasNormals())
        {
            std::cout<<YELLOW<<"[WARNING]::[NormalPCD2Indicator]::the mesh doesn't has normals."<<RESET<<std::endl;
            return std::vector<double>();          
        }
        if(coarse) 
        {
            auto pcd = mesh.GetPointCloud();
            return NormalPCD2Indicator(*pcd, grid_num, origin, voxel_resolution);
        }
        else
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
            std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
                    std::vector<size_t> indices;
                    std::vector<float> dists;
                    kdtree.KnnSearch(p, indices, dists, 5);
                    if(dists.size() > 0 && dists[0] > truncation * truncation)
                    {
                        return 0.0;
                    }
                    double final_distance_abs = 1e7;
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
                            final_distance_abs = std::sqrt(dists[id]);
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
                            if(tmp_dist_abs < final_distance_abs) 
                            {
                                final_distance_abs = tmp_dist_abs;
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
                                    
                                    if(tmp_dist_abs < final_distance_abs)
                                    {
                                        final_distance_abs = tmp_dist_abs;
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
                                    if(tmp_dist_abs < final_distance_abs)
                                    {
                                        final_distance_abs = tmp_dist_abs;
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
                                    if(tmp_dist_abs < final_distance_abs)
                                    {
                                        final_distance_abs = tmp_dist_abs;
                                        float beta = (projected_p - p3).norm() / (p1 - p3).norm();
                                        sign = (((1 - beta) * n3 + beta * n1).dot(tmp_vec) > 0);
                                    }
                                }
                            }
                        }
                    }
                    if(!sign) return 1.0;
                    return 0.0;
                };   
            std::vector<double> sdfs(size_t(grid_num(0)) * grid_num(1) * grid_num(2));
        // unit test
            // geometry::PointCloud vis_pcd;
#pragma omp parallel for
            for(size_t x = 0; x != size_t(grid_num(0)); ++x)
                for(size_t y = 0; y != size_t(grid_num(1)); ++y)
                    for(size_t z = 0; z != size_t(grid_num(2)); ++z)
                    {
                        geometry::Point3 tmp_p = origin + geometry::Point3(x * voxel_resolution, y * voxel_resolution, z * voxel_resolution);
                        sdfs[x * grid_num(1) * grid_num(2) + y * grid_num(2) + z] = get_sdf(tmp_p);
                        // if ( sdf > 0) 
                        // {
                        //     // std::cout<<YELLOW<<"SDF."<<RESET<<std::endl;

                        //     vis_pcd.points.push_back(tmp_p);
                        // }
                    }   

            // vis_pcd.WriteToPLY("./indicator_vis.ply");
            return sdfs;   
        }
    }
    std::vector<double> NormalPCD2Indicator(const geometry::PointCloud &pcd, const geometry::Point3i & grid_num, geometry::Point3 origin, float voxel_resolution, double truncation)
    {
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(pcd.points);
        auto &normals = pcd.normals;
        if(!pcd.HasNormals())
        {
            std::cout<<YELLOW<<"[WARNING]::[NormalPCD2Indicator]::the point cloud doesn't has normals."<<RESET<<std::endl;
            return std::vector<double>();
        }
        std::function<bool (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> bool {
                std::vector<size_t> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(p, indices, dists, 1);
                
                if(indices.size() > 0)
                {
                    if(dists[0] > truncation * truncation) return 0.0;
                    int closest_vid = indices[0];
                    return double((p - pcd.points[closest_vid]).dot(normals[closest_vid]) < 0);
                }
                return 0.0;

            }; 
        std::vector<double> sdfs(size_t(grid_num(0)) * grid_num(1) * grid_num(2));
// unit test
        // geometry::PointCloud vis_pcd;
#pragma omp parallel for
        for(size_t x = 0; x != size_t(grid_num(0)); ++x)
            for(size_t y = 0; y != size_t(grid_num(1)); ++y)
                for(size_t z = 0; z != size_t(grid_num(2)); ++z)
                {
                    geometry::Point3 tmp_p = origin + geometry::Point3(x * voxel_resolution, y * voxel_resolution, z * voxel_resolution);
                    sdfs[x * grid_num(1) * grid_num(2) + y * grid_num(2) + z] = get_sdf(tmp_p);
                    // if ( sdf > 0) 
                    // {
                    //     // std::cout<<YELLOW<<"SDF."<<RESET<<std::endl;

                    //     vis_pcd.points.push_back(tmp_p);
                    // }
                }   

        // vis_pcd.WriteToPLY("./indicator_vis.ply");
        return sdfs;
    
    }
    // void NormalPCD2Indicator(const geometry::PointCloud &pcd, CubeHandler &cube_handler, float voxel_resolution)
    // {
    //     geometry::KDTree<3> kdtree;
    //     kdtree.BuildTree(pcd.points);
    //     auto &normals = pcd.normals;
    //     if(!pcd.HasNormals())
    //     {
    //         std::cout<<YELLOW<<"[WARNING]::[NormalPCD2Indicator]::the point cloud doesn't has normals."<<RESET<<std::endl;
    //         return;
    //     }
    //     // prepare cubes
    //     cube_handler.Clear();
    //     cube_handler.SetVoxelResolution(voxel_resolution);
    //     float truncation = cube_handler.GetTruncation();
    //     float indicator = truncation / 2.5;
    //     std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
    //             std::vector<size_t> indices;
    //             std::vector<float> dists;
    //             kdtree.KnnSearch(p, indices, dists, 1);
    //             bool sign = 1;
    //             if(indices.size() > 0 && std::sqrt(dists[0]) < truncation * 0.5)
    //             {
    //                 int closest_vid = indices[0];
    //                 sign = ((p - pcd.points[closest_vid]).dot(normals[closest_vid]) > 0);
    //                 if(!sign) return -indicator;
    //                 return indicator;
    //             }
    //             return 999;

    //         };        
    //     cube_handler.IntegratePoints(pcd.points, get_sdf);
    //     // for(size_t i = 0; i != mesh.points.size(); ++i)
    //     // {
    //     //     AddCube(cube_handler.GetCubeID(mesh.points[i]));
    //     // }
    //     // std::cout<<BLUE<<"???"<<RESET<<std::endl; 
    //     std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;        
    // }

    geometry::PointCloud Centerline2SurfacePoints(const geometry::Point3List &centers, const std::vector<double> &radius, size_t n_points)
    {
        geometry::Graph graph(centers);
        graph.ConstructEdgeAdaptive(2.5, 8);
        
        // std::vector<geometry::Point3List> all_normals(centers.size());
        geometry::PointCloud base_pcd;
        geometry::PointCloud res_pcd;
        std::vector<std::unordered_map<size_t, size_t>> neighbor_pos(centers.size());
        std::vector<std::vector<size_t>> all_neighbors = graph.GetNeighbors();
        geometry::Vector3 z_axis(0, 0, 1);
        // generate base points using trigonometric function
        double step = 2 * M_PI / n_points;
        for(size_t i = 0; i != n_points; ++i)
        {
            double angle = i * step;
            base_pcd.points.push_back(geometry::Point3(cos(angle), sin(angle), 0.0));
            base_pcd.normals.push_back(geometry::Point3(cos(angle), sin(angle), 0.0));
        }

        for(size_t i = 0; i != centers.size(); ++i)
        {
            const std::vector<size_t> & neighbors = all_neighbors[i]; 
            geometry::Vector3 normal_sum(0,0,0);
            // std::cout<<neighbors.size()<<std::endl;
            geometry::PointCloud tmp_base_r = base_pcd;
            for (size_t j = 0; j != n_points; ++j)
            tmp_base_r.points[j] *= radius[i];
            for(size_t j = 0; j < neighbors.size(); ++j)
            {
                normal_sum += centers[neighbors[j]] - centers[i];
                neighbor_pos[i][neighbors[j]] = j;
            }
            
            for(size_t j = 0; j < neighbors.size(); ++j)
            {
                geometry::PointCloud tmp_base_pcd = tmp_base_r;
                geometry::Vector3 normal = (2 * (centers[neighbors[j]] - centers[i]) - normal_sum ).normalized();
                // all_normals[i].push_back(normal);
                geometry::TransformationMatrix local2global = geometry::TransformationMatrix::Identity();
                local2global.block<3, 3>(0, 0) = 
                    geometry::RotationMatrixBetweenVectors(z_axis, normal);
                local2global.block<3, 1>(0, 3) = centers[i];

                tmp_base_pcd.Transform(local2global);
                // add to res
                res_pcd.MergePCD(tmp_base_pcd);
                // avoid redundant computing.
                if(neighbors.size() <= 2)
                break;
            }
        }    
        return res_pcd;             
    }
    void _UnorderedCenterline2SDF(const geometry::Point3List &centers, const std::vector<double> & radius, CubeHandler &cube_handler, float voxel_resolution, int knn)
    {
        geometry::Graph graph(centers);
        graph.ConstructEdgeAdaptive(2.5, 8);
        // graph.ConstructEdgeRadius(radius);
        // graph.ConstructEdgeKNN(3);
        // graph.ConstructEdgeThreshold(radius);
        std::vector<geometry::Point3List> all_normals(centers.size());
        std::vector<std::unordered_map<size_t, size_t>> neighbor_pos(centers.size());
        std::vector<std::vector<size_t>> all_neighbors = graph.GetNeighbors();
#pragma omp parallel for
        for(size_t i = 0; i != centers.size(); ++i)
        {
            const std::vector<size_t> & neighbors = all_neighbors[i]; 
            geometry::Vector3 normal_sum(0,0,0);
            // std::cout<<neighbors.size()<<std::endl;
            for(size_t j = 0; j < neighbors.size(); ++j)
            {
                normal_sum += centers[neighbors[j]] - centers[i];
                neighbor_pos[i][neighbors[j]] = j;
            }

            for(size_t j = 0; j < neighbors.size(); ++j)
            {
                geometry::Vector3 normal = (2 * (centers[neighbors[j]] - centers[i]) - normal_sum ).normalized();
                all_normals[i].push_back(normal);
            }
            
            // it's better to use a gaussian function to smooth the radius

        }
        geometry::Vector3 z_axis(0, 0, 1);
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(centers);
        cube_handler.Clear();
        cube_handler.SetVoxelResolution(voxel_resolution);

        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double 
        {
            // auto tmp_neighbor_pos = neighbor_pos;
            std::vector<size_t> indices;
            std::vector<float> dists;
            kdtree.KnnSearch(p, indices, dists, knn);
            // p is the center of a voxel
            if((p - centers[indices[0]]).norm() <= DRAGON_EPS)
            {
                return -radius[indices[0]];
            }
            double final_distance = 1e7;
            size_t count = 0;
            for(size_t i = 0; i != indices.size(); ++i)
            {
                size_t s_vid = indices[i];
                const std::vector<size_t> & neighbors = all_neighbors[s_vid]; 

                for(size_t j = 0; j != neighbors.size(); ++j)
                {
                    // can be modified to avoid redundant computing.
                    size_t t_vid = neighbors[j];
                    const geometry::Point3 &c1 = centers[s_vid];
                    const geometry::Point3 &c2 = centers[t_vid];
                    geometry::Point3 &n1 = all_normals[s_vid][j];
                    geometry::Point3 &n2 = all_normals[t_vid][neighbor_pos[t_vid][s_vid]];
                    double r1 = radius[s_vid];
                    double r2 = radius[t_vid];
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
                        // geometry::Point3 tmp_p = geometry::TransformPoint(global2local, p);
                        geometry::Vector3 tmp_n = geometry::TransformNormal(global2local, n);
                        // two intersection
                        geometry::Point3 solution = geometry::Point3::Zero();
                        solution(0) = r1 / std::sqrt( 1 + (tmp_n(0) * tmp_n(0)) / (tmp_n(1) * tmp_n(1)));
                        solution(1) = - tmp_n(0) * solution(0) / tmp_n(1);
                        geometry::Vector3 tmp_help_n = geometry::TransformNormal(global2local, help_n);
                        // if((solution.dot(tmp_help_n) > 0) == (tmp_p.dot(tmp_help_n) > 0))
                        if(solution.dot(tmp_help_n) > 0)
                            inter_p1 = geometry::TransformPoint(local2global, solution);
                        else
                            inter_p1 = geometry::TransformPoint(local2global, -solution);
                    }

                    {
                        geometry::TransformationMatrix local2global = geometry::TransformationMatrix::Identity();
                        local2global.block<3, 3>(0, 0) = 
                            geometry::RotationMatrixBetweenVectors(z_axis, n2);
                        local2global.block<3, 1>(0, 3) = c2; 
                        geometry::TransformationMatrix global2local = local2global.inverse();
                        // geometry::Point3 tmp_p = geometry::TransformPoint(global2local, p);
                        geometry::Vector3 tmp_n = geometry::TransformNormal(global2local, n);

                        // two intersection
                        geometry::Point3 solution = geometry::Point3::Zero();
                        solution(0) = r2  / std::sqrt( 1 + (tmp_n(0) * tmp_n(0)) / (tmp_n(1) * tmp_n(1)) );
                        solution(1) = - tmp_n(0) * solution(0) / tmp_n(1);
                        geometry::Vector3 tmp_help_n = geometry::TransformNormal(global2local, help_n);
                        // if((solution.dot(tmp_help_n) > 0) == (tmp_p.dot(tmp_help_n) > 0))
                        if(solution.dot(tmp_help_n) > 0)
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
                    if(!sign)
                    {
                        if(final_distance > 0) 
                        final_distance = -local_distance;
                        else
                        final_distance -= local_distance;
                        count += 1; 
                    }
                    else if(local_distance < final_distance) final_distance = local_distance;

                    // if(local_distance < 0) std::cout<<local_distance<<std::endl;
                }
            }
            // if(final_distance <= 0) std::cout<<final_distance / 3 <<std::endl;
            if(count > 0) return final_distance / count;
            double dist_to_ball = geometry::Distance(centers[indices[0]], p) - radius[indices[0]];
            if(dist_to_ball < final_distance)
            final_distance = dist_to_ball;
            return final_distance;
        };    
        cube_handler.IntegratePoints(centers, get_sdf);
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;   
    }
    void _OrderedCenterLine2SDF(const geometry::Point3List &centers, const std::vector<double> &radius, CubeHandler &cube_handler, float voxel_resolution, int knn)
    {
        geometry::Point3List normals;
        normals.resize(centers.size());
        for(size_t i = 0; i != centers.size(); ++i)
        {
            if(i == 0) normals[i] = (centers[1] - centers[0]).normalized();
            else if(i == centers.size() - 1) normals[i] = (centers[i] - centers[i-1]).normalized();
            else normals[i] = (centers[i + 1] - centers[i - 1]).normalized();
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
                kdtree.KnnSearch(p, indices, dists, knn);
                double final_distance = 1e7;
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
                        // geometry::Point3 tmp_p = geometry::TransformPoint(global2local, p);
                        geometry::Vector3 tmp_n = geometry::TransformNormal(global2local, n);
                        // two intersection
                        geometry::Point3 solution = geometry::Point3::Zero();
                        solution(0) = r1 / std::sqrt( 1 + (tmp_n(0) * tmp_n(0)) / (tmp_n(1) * tmp_n(1)));
                        solution(1) = - tmp_n(0) * solution(0) / tmp_n(1);
                        geometry::Vector3 tmp_help_n = geometry::TransformNormal(global2local, help_n);
                        // if((solution.dot(tmp_help_n) > 0) == (tmp_p.dot(tmp_help_n) > 0))
                        if(solution.dot(tmp_help_n) > 0)
                            inter_p1 = geometry::TransformPoint(local2global, solution);
                        else
                            inter_p1 = geometry::TransformPoint(local2global, -solution);
                        // geometry::Point3 test_p;
                        // geometry::CheckPointProjectionInTriangle(tmp_p, solution, geometry::Point3(0, 0, 0), 
                        //     geometry::TransformPoint(global2local, c2), test_p);

                        // std::cout<<test_p.transpose()<<" "<<tmp_p.transpose()<<std::endl;
                    }

                    {
                        geometry::TransformationMatrix local2global = geometry::TransformationMatrix::Identity();
                        local2global.block<3, 3>(0, 0) = 
                            geometry::RotationMatrixBetweenVectors(z_axis, n2);
                        local2global.block<3, 1>(0, 3) = c2; 
                        geometry::TransformationMatrix global2local = local2global.inverse();
                        // geometry::Point3 tmp_p = geometry::TransformPoint(global2local, p);
                        geometry::Vector3 tmp_n = geometry::TransformNormal(global2local, n);

                        // two intersection
                        geometry::Point3 solution = geometry::Point3::Zero();
                        solution(0) = r2  / std::sqrt( 1 + (tmp_n(0) * tmp_n(0)) / (tmp_n(1) * tmp_n(1)) );
                        solution(1) = - tmp_n(0) * solution(0) / tmp_n(1);
                        geometry::Vector3 tmp_help_n = geometry::TransformNormal(global2local, help_n);
                        // if((solution.dot(tmp_help_n) > 0) == (tmp_p.dot(tmp_help_n) > 0))
                        if(solution.dot(tmp_help_n) > 0)
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
                    if(local_distance < final_distance) final_distance = local_distance;
                    // if(local_distance < 0) std::cout<<local_distance<<std::endl;
                }
                double dist_to_ball = geometry::Distance(centers[indices[0]], p) - radius[indices[0]];
                if(dist_to_ball < final_distance)
                final_distance = dist_to_ball;
                return final_distance;

            };    
        cube_handler.IntegratePoints(centers, get_sdf);
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;   
    }

    // Fast computation
    void _UnorderedCenterline2SDFFast(const geometry::Point3List &centers, const std::vector<double> & radius, CubeHandler &cube_handler, float voxel_resolution, int knn)
    {
        geometry::Graph graph(centers);
        graph.ConstructEdgeAdaptive(2.5, 8);
        // graph.ConstructEdgeRadius(radius);
        // graph.ConstructEdgeKNN(3);
        // graph.ConstructEdgeThreshold(radius);


        std::vector<std::vector<size_t>> all_neighbors = graph.GetNeighbors();
        geometry::Vector3 z_axis(0, 0, 1);
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(centers);
        cube_handler.Clear();
        cube_handler.SetVoxelResolution(voxel_resolution);

        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double 
        {
            // auto tmp_neighbor_pos = neighbor_pos;
            std::vector<size_t> indices;
            std::vector<float> dists;
            kdtree.KnnSearch(p, indices, dists, knn);
            // p is the center of a voxel
            if(geometry::Distance(centers[indices[0]], p) <= DRAGON_EPS)
            {
                return -radius[indices[0]];
            }
            double final_distance = 1e7;
            size_t count = 0;
            for(size_t i = 0; i != indices.size(); ++i)
            {
                size_t s_vid = indices[i];
                const std::vector<size_t> & neighbors = all_neighbors[s_vid]; 

                for(size_t j = 0; j != neighbors.size(); ++j)
                {
                    // can be modified to avoid redundant computing.
                    size_t t_vid = neighbors[j];
                    const geometry::Point3 &c1 = centers[s_vid];
                    const geometry::Point3 &c2 = centers[t_vid];
                    double r1 = radius[s_vid];
                    double r2 = radius[t_vid];
                    // project p to the centerline       
                    geometry::Point3 projected_p;
                    bool on_line_segment = geometry::CheckPointProjectionOnLineSegment(p, c1, c2, projected_p);    
                    if(on_line_segment)
                    {
                        double d1 = geometry::Distance(c1, projected_p);
                        double d2 = geometry::Distance(c2, projected_p);
                        double projected_radius = (d1 * r2 + d2 * r1 )/ (d1 + d2);
                        double tmp_distance = geometry::Distance(p, projected_p) - projected_radius;
                        if(tmp_distance < 0)
                        {
                            if(final_distance > 0) 
                            final_distance = tmp_distance;
                            else
                            final_distance += tmp_distance;
                            count += 1; 
                        }
                        else if(tmp_distance < final_distance) final_distance = tmp_distance;
                        if(tmp_distance < final_distance) final_distance = tmp_distance;
                    }
                }
            }
            if(count > 0)
            final_distance /= count;
            double dist_to_ball = geometry::Distance(centers[indices[0]], p) - radius[indices[0]];
            if(dist_to_ball < final_distance)
            final_distance = dist_to_ball;
            return final_distance;

        };    
        cube_handler.IntegratePoints(centers, get_sdf);
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;   
    }

    void _OrderedCenterLine2SDFFast(const geometry::Point3List &centers, const std::vector<double> &radius, CubeHandler &cube_handler, float voxel_resolution, int knn)
    {
        geometry::Vector3 z_axis(0, 0, 1);
        geometry::KDTree<3> kdtree;
        kdtree.BuildTree(centers);
        cube_handler.Clear();
        cube_handler.SetVoxelResolution(voxel_resolution);

        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
                std::vector<size_t> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(p, indices, dists, knn);
                double final_distance = 1e7;
                std::set<int> line_seg_ids;
                for(size_t i = 0; i != indices.size(); ++i)
                {
                    if(indices[i] > 0) line_seg_ids.insert(indices[i] - 1);
                    if(indices[i] < centers.size() - 1) line_seg_ids.insert(indices[i]);
                }
                int count = 0;
                for(auto iter = line_seg_ids.begin(); iter != line_seg_ids.end(); ++iter)
                {
                    int lid = *iter;
                    const geometry::Point3 &c1 = centers[lid];
                    const geometry::Point3 &c2 = centers[lid + 1];
                    double r1 = radius[lid];
                    double r2 = radius[lid + 1];
                    // project p to the centerline       
                    geometry::Point3 projected_p;
                    bool on_line_segment = geometry::CheckPointProjectionOnLineSegment(p, c1, c2, projected_p);    
                    if(on_line_segment)
                    {
                        double d1 = geometry::Distance(c1, projected_p);
                        double d2 = geometry::Distance(c2, projected_p);
                        double projected_radius = (d1 * r2 + d2 * r1 )/ (d1 + d2);
                        double tmp_distance = geometry::Distance(p, projected_p) - projected_radius;
                        if(tmp_distance < 0)
                        {
                            if(final_distance > 0) 
                            final_distance = tmp_distance;
                            else
                            final_distance += tmp_distance;
                            count += 1; 
                        }
                        else if(tmp_distance < final_distance) final_distance = tmp_distance;
                    }
                }
                if(count > 0)
                final_distance /= count;
                double dist_to_ball = geometry::Distance(centers[indices[0]], p) - radius[indices[0]];
                if(dist_to_ball < final_distance)
                final_distance = dist_to_ball;
                return final_distance;

            };    
        cube_handler.IntegratePoints(centers, get_sdf);
        std::cout<<BLUE<<"[INFO]::[Mesh2SDF]::Generate "<<cube_handler.Size()<<" cubes."<<RESET<<std::endl;   
    }

    void CenterLine2SDF(const geometry::Point3List &centers, const std::vector<double> &radius, CubeHandler &cube_handler, float voxel_resolution, bool ordered, int knn, bool fast_computation)
    {
        if(fast_computation)
        {
            if(ordered) _OrderedCenterLine2SDFFast(centers, radius, cube_handler, voxel_resolution, knn);
            else _UnorderedCenterline2SDFFast(centers, radius, cube_handler, voxel_resolution, knn);
        }
        else
        {
            if(ordered) _OrderedCenterLine2SDF(centers, radius, cube_handler, voxel_resolution, knn);
            else _UnorderedCenterline2SDF(centers, radius, cube_handler, voxel_resolution, knn);            
        }
    }
}
}