#include "Geometry/TriangleMesh/Processing/MinimalSurface.h"
#include "Geometry/TriangleMesh/Processing/Smoothing.h"
#include "Visualization/Visualizer.h"
using namespace dragon;
int main(int argc, char* argv[]) 
{
    if(argc != 2)
    {
        std::cout << "Usage: GenerateMinimalSurface [filename.obj]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromPLY(argv[1]);
    auto minimal_surface = geometry::MinimalSurfaceLocal(mesh, 0.2, 1000);
    auto laplacian_surface = geometry::LaplacianSmooting(mesh);
    // if(!mesh.HasNormals())
    //     mesh.ComputeNormals();
    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();

    minimal_surface->WriteToPLY("./minimal_surface.ply");
    laplacian_surface->WriteToPLY("./laplacian_surface.ply");
    return 0;
}