#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
#include "Geometry/Structure/Graph.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout << "Usage: SkeletonGraph2SDF [input_filename] [output_filename] [voxel resolution = 0.01] [scaling = -1] [fast_computation = 1]"<<std::endl;
        return 0;        
    }

    geometry::TriangleMesh generated_mesh;
    geometry::SkeletonGraph sgraph;
    sgraph.LoadFromPLY(argv[1]);
    std::string output_filename = argv[2];
    double scale = -1;
    // double radius_factor = -1.0;
    float voxel_resolution = 0.01;
    int fast_computation = 1;
    if(argc > 3)
    voxel_resolution = std::atof(argv[3]);
    if(argc > 4)
    scale = std::atof(argv[4]);
    if(argc > 5)
    fast_computation = std::atoi(argv[5]);
    // scaling
    // why is scaling so important?
    if(scale <= 0)
    {
        geometry::BoundingBox bb;
        for(size_t i = 0; i != sgraph.vertices.size(); ++i)
        bb.AddPoint(sgraph.vertices[i]);
        scale = 1 / (std::max(bb.y_max - bb.y_min, std::max(bb.x_max - bb.x_min, bb.z_max - bb.z_min)));
        // std::cout<<scale<<" "<<bb.y_max <<" "<< bb.y_min<<" " <<bb.x_max <<" "<< bb.x_min <<" "<<bb.z_max <<" "<< bb.z_min <<std::endl;
        
    }
    for(size_t i = 0; i != sgraph.vertices.size(); ++i)
    {
        sgraph.vertices[i] *= scale;
        sgraph.radius[i] *= scale;
    }

    reconstruction::CubeHandler cube_handler;
    cube_handler.SetTruncation(voxel_resolution * 5);

    cube_handler.SetVoxelResolution(voxel_resolution);
    reconstruction::SkeletonGraph2SDF(sgraph, cube_handler, voxel_resolution, 2, fast_computation);


    cube_handler.ExtractTriangleMesh(generated_mesh);
    auto simplified_mesh = geometry::mesh::ClusteringDecimation(generated_mesh, voxel_resolution);
    auto res_mesh = geometry::mesh::ToManifoldMesh(*simplified_mesh);
    res_mesh->Scale(1.0 / scale);
    res_mesh-> WriteToPLY(output_filename);

    return 0;
}
