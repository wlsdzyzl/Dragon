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
        std::cout << "Usage: GenerateMinimalSurface [filename.obj]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromOBJ(argv[1]);
    geometry::HalfEdge he; 
    he.FromTriangleMesh(mesh);
    he.CheckBorder();
    geometry::Point3List mean_curvature_vectors;
    geometry::ComputeMeanCurvature(he, mean_curvature_vectors);
    // if(!mesh.HasNormals())
    //     mesh.ComputeNormals();
    std::vector<double> mean_curvature;
    double min_c = std::numeric_limits<double>::max();
    double max_c = std::numeric_limits<double>::min();
    for(size_t i = 0; i != mean_curvature_vectors.size(); ++i)
    {
        mean_curvature.push_back(mean_curvature_vectors[i].norm());
        if(mean_curvature.back() > max_c) {max_c = mean_curvature.back();}
        if(mean_curvature.back() < min_c) min_c = mean_curvature.back();
    }
    
    for(size_t i = 0; i != he.vertices.size(); ++i)
    {
        
        // if(mean_curvature[i] > 1)
        // std::cout<<i<<" "<<mean_curvature[i]<<std::endl;
        he.vertices[i].color = ColorRemapping(min_c, max_c, mean_curvature[i]);
    }
    he.has_colors = true;
    geometry::TriangleMesh mapping_mesh;
    he.ToTriangleMesh(mapping_mesh);

    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();
    std::sort(mean_curvature.begin(), mean_curvature.end());
    mapping_mesh.WriteToPLY("./mean_curvature.ply");
    return 0;
}