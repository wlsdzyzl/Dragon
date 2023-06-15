#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
#include "cnpy.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc < 5)
    {
        std::cout << "Usage: MeshIndicatorTest [filename] [grid_num_x] [grid_num_y] [grid_num_z] [voxel resolution = 0.01] [origin_x = 0] [origin_y = 0] [origin_z = 0]"<<std::endl;
        return 0;        
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromFile(argv[1]);
    //
    mesh.ComputeNormals();
    geometry::Point3i grid_num(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
    geometry::Point3 origin;
    double voxel_resolution = 0.01;
    if(argc > 5)
    voxel_resolution = atof(argv[5]);
    if(argc >=9)
    origin = geometry::Point3(atof(argv[6]), atof(argv[7]), atof(argv[8]));
    

    std::vector<double> indicator = reconstruction::Mesh2Indicator(mesh, grid_num, origin, voxel_resolution);
    cnpy::npy_save("mask.npy",&indicator[0],{size_t(grid_num(2)), size_t(grid_num(1)), size_t(grid_num(0))},"w");
    return 0;
}