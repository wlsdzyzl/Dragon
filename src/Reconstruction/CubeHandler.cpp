#include "CubeHandler.h"
#include "Tool/MultiThreads.h"
#include "MarchingCube.h"
#include "Geometry/Structure/BoundingBox.h"
#include <omp.h>
#define SINGLE_THREAD 0
namespace dragon
{
namespace reconstruction
{
    void CubeHandler::ExtractTriangleMesh(geometry::TriangleMesh &mesh)
    {
        /* 
        auto iter = cube_map.begin();
        for(size_t i = 0;i!=cube_map.size()-2; ++i)
        iter++;
        GenerateMeshByCube(iter->first,mesh);
         */
        //
#if SINGLE_THREAD   
        //single thread
        for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
        {
            GenerateMeshByCube(iter->first,mesh);
        }
        std::cout<<GREEN<<"[ExtractTriangleMesh]::[Info]::Finish mesh extraction( single thread)."<<RESET<<std::endl;
#else
        //Multi threads 
        std::vector<CubeID> cube_ids;
        std::vector<geometry::TriangleMesh> meshes;
        for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
        {
            cube_ids.push_back(iter->first);
            meshes.push_back(geometry::TriangleMesh());
        }

        std::function<void(const CubeID &, geometry::TriangleMesh &)> f = 
        [&](const CubeID & cube_id, geometry::TriangleMesh & _mesh)
        {
            GenerateMeshByCube(cube_id, _mesh);
        };
        tool::MultiThreads<CubeID, geometry::TriangleMesh>(cube_ids, meshes, f);
        mesh.LoadFromMeshes(meshes);
        std::cout<<GREEN<<"[ExtractTriangleMesh]::[Info]::Finish mesh extraction( multi-threads)."<<RESET<<std::endl;
#endif    
    }
    void CubeHandler::GenerateMeshByCube(const CubeID &cube_id, geometry::TriangleMesh &mesh)
    {
        std::vector<TSDFVoxel> corner_tsdf(8);
        geometry::Point3List corner_voxel(8);

        //std::cout<<cube_id<<std::endl;
        for(size_t x = 0 ; x != CUBE_SIZE; ++x)
        {
            for(size_t y = 0 ; y != CUBE_SIZE; ++y)
            {
                for(size_t z = 0 ; z != CUBE_SIZE; ++z)
                {        
                    //int voxel_id = x + y * CUBE_SIZE + z * CUBE_SIZE * CUBE_SIZE;
                    int index = (x == (CUBE_SIZE-1) ) + ((y == (CUBE_SIZE-1))<<1) + ((z == (CUBE_SIZE -1)) << 2);
                    Eigen::Vector3i cube_offset =  c_para.NeighborCubeIDOffset[index];
                    //std::cout<<"CUBE OFFSET: "<<index<<std::endl;
                    bool all_neighbors_observed = true;
                    for( int i = 0; i!= 8; ++i)
                    {
                        Eigen::Vector3i xyz_offset = c_para.CornerXYZOffset.block<3,1>(0,i);
                        Eigen::Vector3i neighbor_cube_id = cube_id +Eigen::Vector3i( xyz_offset(0) & cube_offset(0), xyz_offset(1) & cube_offset(1), xyz_offset(2)&cube_offset(2));
                        Eigen::Vector3i vertex_offset = Eigen::Vector3i((x+xyz_offset(0))%CUBE_SIZE, (y+xyz_offset(1))%CUBE_SIZE, (z+xyz_offset(2))%CUBE_SIZE);
                        int neighbor_voxel_id = vertex_offset(0) + vertex_offset(1) * CUBE_SIZE + vertex_offset(2) * CUBE_SIZE * CUBE_SIZE;                        
                        if(!HasCube(neighbor_cube_id)) 
                        {
                            all_neighbors_observed = false;
                            break;
                        }
                        corner_voxel[i] = cube_map[neighbor_cube_id].GetOrigin(c_para) + c_para.VoxelCentroidOffSet[neighbor_voxel_id]; 
                        corner_tsdf[i] = cube_map[neighbor_cube_id].voxels[neighbor_voxel_id];
                        // sdf value is large than 1
                        if(!corner_tsdf[i].IsValid()) 
                        {
                            all_neighbors_observed = false;
                            break;
                        }
                    }
                    if(all_neighbors_observed)  MarchingCube(corner_voxel, corner_tsdf, mesh);
                }
            }
        }

        return;
    }
    void CubeHandler::PrepareCubes(const geometry::Point3List &points, std::vector<CubeID> &cube_id_list, std::function<double(geometry::Point3)> get_sdf)
    {
        geometry::BoundingBox bb;
        for(size_t i = 0; i != points.size(); ++i)
        {
            bb.AddPoint(points[i]);
        }
        geometry::Point3 max_point(bb.x_max + truncation, bb.y_max + truncation, bb.z_max + truncation), 
            min_point(bb.x_min - truncation, bb.y_min - truncation, bb.z_min - truncation);
        CubeID max_cube_id = GetCubeID(max_point);
        CubeID min_cube_id = GetCubeID(min_point);        

        cube_id_list.clear();
        std::vector<int> voxel_index
             {0, CUBE_SIZE -1, (CUBE_SIZE-1)*CUBE_SIZE, (CUBE_SIZE-1)*CUBE_SIZE + CUBE_SIZE-1,
            CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1), CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE-1,
            CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE * (CUBE_SIZE-1),
            CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE-1};
        
        double cube_resolution = c_para.VoxelResolution * CUBE_SIZE;

        for(int i = min_cube_id(0)-1; i<= max_cube_id(0)+1; ++i)
        {
            for(int j = min_cube_id(1)-1; j<= max_cube_id(1)+1; ++j)
            {
                for(int k = min_cube_id(2)-1; k<= max_cube_id(2)+1; ++k)
                {
                    
                    double min_sdf = std::numeric_limits<double>::max();
                    bool same_sign = true;
                    bool start_sign = true;
                    for(size_t c = 0; c!= 8; ++c)
                    {
                        geometry::Point3 voxel = geometry::Point3(i*cube_resolution, j*cube_resolution, 
                            k*cube_resolution) + c_para.VoxelCentroidOffSet[voxel_index[c]];
                        double sdf = get_sdf(voxel);
                        if(c == 0 && sdf < 0) start_sign = false;
                        if( c > 0 && ((sdf > 0) != start_sign)) same_sign = false;
                        if(min_sdf > std::fabs(sdf))
                            min_sdf =std::fabs(sdf);
                    }

                    if(min_sdf < truncation || !same_sign)
                    {
                        CubeID cube_id = CubeID(i,j,k);
                        // bool is_new_cube=false;
                        if(cube_map.find(cube_id) == cube_map.end())
                        {
                            // is_new_cube = true;
                            cube_map[cube_id] = VoxelCube(cube_id);
                        }
                        cube_id_list.push_back(cube_id);
                    }
                    //std::cout<<k<<std::endl;
                }
            }
        }        
    }
    void CubeHandler::IntegratePoints(const geometry::Point3List &points, std::function<double(geometry::Point3)> get_sdf)
    {
        std::vector<CubeID> cube_id_list;
        PrepareCubes(points, cube_id_list, get_sdf);
        std::cout<<cube_id_list.size()<<" cubes are waitting to be integrated, which are "<< cube_id_list.size() * 512 << " voxels."<<std::endl;
        for(size_t i = 0; i != cube_id_list.size(); ++i)
        {
            CubeID &cube_id = cube_id_list[i];
            VoxelCube &cube = cube_map[cube_id];
            if(i % 10 == 0) std::cout<<"progress: "<<(i*100.0)/cube_id_list.size()<<"%"<<std::endl;
       
            for(size_t x = 0 ; x != CUBE_SIZE; ++x)
            {
                for(size_t y = 0 ; y != CUBE_SIZE; ++y)
                {
                    // #pragma omp parallel for     
                    for(int z = 0 ; z != CUBE_SIZE; ++z)
                    {   
                        int voxel_id = x + y * CUBE_SIZE + z * CUBE_SIZE * CUBE_SIZE;
                        geometry::Point3 g_point = c_para.GetGlobalPoint(cube_id, voxel_id);
                        double sdf = get_sdf(g_point);
                        cube.voxels[voxel_id].sdf = sdf;
                        if(std::fabs(sdf) < truncation)
                        {
                            cube.voxels[voxel_id].weight = 1;
                            
                        }
                        if(std::fabs(sdf) > max_sdf)
                        max_sdf = std::fabs(sdf);
                    }
                }
            }
        }
    }
        std::shared_ptr<geometry::PointCloud> CubeHandler::GetPointCloud() const
        {
            geometry::PointCloud pcd;
            for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
            {
                //std::cout<<iter->first<<std::endl;
                for(size_t x = 0 ; x != CUBE_SIZE; ++x)
                {
                    for(size_t y = 0 ; y != CUBE_SIZE; ++y)
                    {
                        for(size_t z = 0 ; z != CUBE_SIZE; ++z)
                        {       
                            size_t voxel_id = x + y * CUBE_SIZE + z * CUBE_SIZE * CUBE_SIZE;
                            // std::cout<<iter->second.voxels[voxel_id].weight<<" "<<std::fabs(iter->second.voxels[voxel_id].sdf)<<std::endl;
                            if(iter->second.voxels[voxel_id].weight !=0 && std::fabs(iter->second.voxels[voxel_id].sdf) <= truncation / 2)
                            {
                                    float sdf = iter->second.voxels[voxel_id].sdf / (truncation / 2);
                                    pcd.points.push_back(iter->second.GetOrigin(c_para) + c_para.VoxelCentroidOffSet[voxel_id]);
                                    if(sdf > 0)
                                    pcd.colors.push_back(geometry::Point3(1, 1 - sdf, 1 - sdf));
                                    else 
                                    pcd.colors.push_back(geometry::Point3(1 + sdf, 1 + sdf, 1)); 
                                    // if(sdf > 0)
                                    // pcd.colors.push_back(geometry::Point3(1, 0, 0));
                                    // else 
                                    // pcd.colors.push_back(geometry::Point3(0, 0, 0)); 
                            }
                        }
                    }
                }
            }   
            return std::make_shared<geometry::PointCloud>(pcd);     
        }
}
}