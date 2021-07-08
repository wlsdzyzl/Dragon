#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout << "Usage: Mesh2SDFTest [filename] [voxel resolution = 0.01]"<<std::endl;
        return 0;        
    }
    geometry::TriangleMesh mesh, generated_mesh;
    mesh.LoadFromFile(argv[1]);
    
    // scaling
    geometry::BoundingBox bb;
    for(size_t i = 0; i != mesh.points.size(); ++i)
    bb.AddPoint(mesh.points[i]);
    double scale = 1 / (std::max(bb.y_max - bb.y_min, std::max(bb.x_max - bb.x_min, bb.z_max - bb.z_min)));
    mesh.Scale(scale);
    float voxel_resolution = 0.01;
    if(argc > 2)
    voxel_resolution = std::atof(argv[2]);

    reconstruction::CubeHandler cube_handler;
    reconstruction::Mesh2SDF(mesh, cube_handler, voxel_resolution);
    auto cube_pcd = cube_handler.GetPointCloud();
    cube_pcd->Scale(1 / scale);
    cube_pcd->WriteToPLY("./tsdf.ply");
    cube_handler.ExtractTriangleMesh(generated_mesh);
    generated_mesh.Scale(1 / scale);
    generated_mesh.WriteToPLY("./generated_mesh.ply");
    return 0;
}