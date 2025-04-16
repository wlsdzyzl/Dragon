#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
#include "cnpy.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc < 5)
    {
        std::cout << "Usage: MeshIndicatorTest [filename] [output_path] [normalize = 1] [voxel resolution = 0.01] [truncation = 1e6] [coarse_computing = 1] [grid_num_x = 1 / voxel_resolution] [grid_num_y = 1 / voxel_resolution] [grid_num_z = = 1 / voxel_resolution] "<<std::endl;
        return 0;        
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromFile(argv[1]);
    mesh.ComputeNormals();

    std::string output_file = argv[2];
    bool normalize = atoi(argv[3]);
    if(normalize) mesh.Normalize();
    
    geometry::Point3 origin; // 0,0,0
    
    double voxel_resolution = 0.01;
    
    double truncation = 1e6;
    bool coarse_computing = 1;
    if(argc > 4)
    voxel_resolution = atof(argv[4]);
    geometry::Point3i grid_num = geometry::Point3i(int(1 / voxel_resolution), int(1 / voxel_resolution), int(1 / voxel_resolution)); 
    if(argc > 5)
    truncation = atof(argv[5]);
    if(argc > 6)
    coarse_computing = atoi(argv[6]);
    if(argc > 7)
    grid_num = geometry::Point3i(atoi(argv[7]), atoi(argv[8]), atoi(argv[9]));
    
    geometry::ScalarList indicator = reconstruction::Mesh2Indicator(mesh, grid_num, origin, voxel_resolution, truncation, coarse_computing);

    cnpy::npy_save(output_file, &indicator[0],{size_t(grid_num(0)), size_t(grid_num(1)), size_t(grid_num(2))},"w");
    std::cout<<"save to " <<output_file<<" with size of ("<<grid_num[0]<<", "<<grid_num[1]<<", "<<grid_num[2]<<")"<<std::endl;
    return 0;
}