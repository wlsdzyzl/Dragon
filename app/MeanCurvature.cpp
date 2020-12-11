#include "Geometry/TriangleMesh/Processing/Curvature.h"
#include "Visualization/Visualizer.h"
#include "Tool/ColorMapping.h"
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
        std::cout << "Usage: GenerateMinimalSurface [filename]"<<std::endl;
        return 0;
    }
    geometry::mesh::TriangleMesh mesh;
    mesh.LoadFromFile(argv[1]);
    geometry::mesh::HalfEdge he; 
    he.FromTriangleMesh(mesh);
    he.CheckBorder();
    std::vector<double> mean_curvatures;
    geometry::mesh::ComputeMeanCurvature(he, mean_curvatures);
    // if(!mesh.HasNormals())
    //     mesh.ComputeNormals();
    // double min_c = std::numeric_limits<double>::max();
    // double max_c = std::numeric_limits<double>::min();
    // for(size_t i = 0; i != mean_curvatures.size(); ++i)
    // {
    //     // if(mean_curvatures[i] < -3) std::cout<<i<<" "<<mean_curvatures[i]<<std::endl;
    //     if(mean_curvatures[i] > max_c) max_c = mean_curvatures[i];
    //     if(mean_curvatures[i] < min_c) min_c = mean_curvatures[i];
    // }
    // for(size_t i = 0; i != he.vertices.size(); ++i)
    // {   
    //     he.vertices[i].color = ColorRemapping(min_c, max_c, mean_curvatures[i]);
    // }
    geometry::Point3List colors;
    tool::ColorRemapping(mean_curvatures, colors);
    for(size_t i = 0; i != he.vertices.size(); ++i)
    {   
        he.vertices[i].color = colors[i];
    }    
    he.has_colors = true;
    geometry::mesh::TriangleMesh mapping_mesh;
    he.ToTriangleMesh(mapping_mesh);

    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();
    mapping_mesh.WriteToPLY("./mean_curvature.ply");
    return 0;
}