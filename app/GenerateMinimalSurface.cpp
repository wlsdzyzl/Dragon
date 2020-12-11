#include "Geometry/TriangleMesh/Processing/Smoothing.h"
#include "Geometry/TriangleMesh/Processing/MeshParameterization.h"
#include "Visualization/Visualizer.h"
using namespace dragon;
int main(int argc, char* argv[]) 
{
    if(argc != 2)
    {
        std::cout << "Usage: GenerateMinimalSurface [filename]"<<std::endl;
        return 0;
    }
    geometry::mesh::TriangleMesh mesh;
    
    mesh.LoadFromFile(argv[1]);
    auto minimal_surface = geometry::mesh::LocalLaplacianSmooting(mesh, 0.2, 1000);
    auto laplacian_surface = geometry::mesh::NaiveLaplacianSmooting(mesh);
    auto global_minimal_surface = geometry::mesh::GlobalLaplacianSmooting(mesh, 0);
    auto uv_map = geometry::mesh::MeshParameterization(mesh);
    // if(!mesh.HasNormals())
    //     mesh.ComputeNormals();
    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();

    minimal_surface->WriteToPLY("./minimal_surface.ply");
    laplacian_surface->WriteToPLY("./laplacian_surface.ply");
    global_minimal_surface->WriteToPLY("./global_minimal_surface.ply");
    uv_map->WriteToPLY("./uv_map.ply");
    return 0;
}