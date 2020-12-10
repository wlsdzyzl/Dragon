#include "Geometry/TriangleMesh/Processing/Curvature.h"
#include "Visualization/Visualizer.h"
using namespace dragon;
geometry::Point3 ColorRemapping(double min_c, double max_c, double tmp)
{
    double middle_c = (min_c + max_c) / 2;
    if(tmp < middle_c)
    return geometry::Point3((tmp - min_c) / (middle_c - min_c), (tmp - min_c) / (middle_c - min_c) , 1 );
    else 
    return geometry::Point3(1 , 1 - (tmp - middle_c)/ (max_c - middle_c), 1 - (tmp - middle_c)/ (max_c - middle_c));
}
int main(int argc, char* argv[]) 
{
    if(argc != 2)
    {
        std::cout << "Usage: GaussCurvature [filename]"<<std::endl;
        return 0;
    }
    geometry::mesh::TriangleMesh mesh;
    mesh.LoadFromFile(argv[1]);
    geometry::mesh::HalfEdge he; 
    he.FromTriangleMesh(mesh);
    std::cout<<"Build HalfEdge!"<<std::endl;
    he.CheckBorder();
    std::vector<double> gauss_curvatures;
    geometry::mesh::ComputeGaussCurvature(he, gauss_curvatures);

    double min_c = std::numeric_limits<double>::max();
    double max_c = std::numeric_limits<double>::min();
    for(size_t i = 0; i != gauss_curvatures.size(); ++i)
    {
        // if(gauss_curvatures[i] < -3) std::cout<<i<<" "<<gauss_curvatures[i]<<std::endl;
        if(gauss_curvatures[i] > max_c) max_c = gauss_curvatures[i];
        if(gauss_curvatures[i] < min_c) min_c = gauss_curvatures[i];
    }
    for(size_t i = 0; i != he.vertices.size(); ++i)
    {   
        he.vertices[i].color = ColorRemapping(min_c, max_c, gauss_curvatures[i]);
    }
    geometry::mesh::TriangleMesh mapping_mesh;
    he.ToTriangleMesh(mapping_mesh);

    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();

    mapping_mesh.WriteToPLY("./gauss_curvature.ply");
    return 0;
}