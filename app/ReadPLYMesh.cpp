#include "Visualization/Visualizer.h"
using namespace dragon;
int main(int argc, char* argv[]) 
{
    
    if(argc != 2)
    {
        std::cout << "Usage: ReadPLYMesh [filename.ply]"<<std::endl;
        return 0;
    }
    geometry::mesh::TriangleMesh mesh;
    mesh.LoadFromOBJ(argv[1]);

    // if(!mesh.HasNormals())
    //     mesh.ComputeNormals();
    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();
    mesh.WriteToPLY("./dragon.ply");
    return 0;
}