#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout << "Usage: NormalPCD2IndicatorTest [filename] [voxel resolution = 0.01]"<<std::endl;
        return 0;        
    }
    geometry::PointCloud pcd;
    pcd.LoadFromFile(argv[1]);
    
    // scaling
    geometry::BoundingBox bb;
    for(size_t i = 0; i != pcd.points.size(); ++i)
    bb.AddPoint(pcd.points[i]);
    double scale = 1 / (std::max(bb.y_max - bb.y_min, std::max(bb.x_max - bb.x_min, bb.z_max - bb.z_min)));
    pcd.Scale(scale);
    float voxel_resolution = 0.01;
    if(argc > 2)
    voxel_resolution = std::atof(argv[2]);

    reconstruction::CubeHandler cube_handler;
    reconstruction::NormalPCD2Indicator(pcd, cube_handler, voxel_resolution);
    auto cube_pcd = cube_handler.GetPointCloud();
    cube_pcd->WriteToPLY("./tsdf.ply");
    return 0;
}