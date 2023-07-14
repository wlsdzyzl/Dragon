#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
#include "cnpy.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc < 5)
    {
        std::cout << "Usage: MeshIndicatorTest [filename] [output_path] [grid_num_x] [grid_num_y] [grid_num_z] [voxel resolution = 0.01] [truncation = 1e6] [coarse_computing = 1]  [origin_x = 0] [origin_y = 0] [origin_z = 0]"<<std::endl;
        return 0;        
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromFile(argv[1]);
    mesh.ComputeNormals();
    std::string output_file = argv[2];
    geometry::Point3i grid_num(atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
    geometry::Point3 origin;
    double voxel_resolution = 0.01;
    double truncation = 1e6;
    bool coarse_computing = 1;
    if(argc > 6)
    voxel_resolution = atof(argv[6]);
    if(argc > 7)
    truncation = atof(argv[7]);
    if(argc > 8)
    coarse_computing = atoi(argv[8]);
    if(argc > 11)
    origin = geometry::Point3(atof(argv[9]), atof(argv[10]), atof(argv[11]));
    

    std::vector<double> indicator = reconstruction::Mesh2Indicator(mesh, grid_num, origin, voxel_resolution, truncation, coarse_computing);
    cnpy::npy_save(output_file, &indicator[0],{size_t(grid_num(0)), size_t(grid_num(1)), size_t(grid_num(2))},"w");
    return 0;
}