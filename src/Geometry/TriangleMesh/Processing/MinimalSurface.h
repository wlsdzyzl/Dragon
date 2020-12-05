// generate minimal surface
#include "../HalfEdge.h"
#include "./Curvature.h"
#define MAX_ITERATION 1000
namespace dragon
{
namespace geometry
{
    std::shared_ptr<TriangleMesh> ComputeMinimalSurface(const TriangleMesh &mesh, double lambda = 0.01)
    {
        HalfEdge he; 
        he.FromTriangleMesh(mesh);
        std::cout<<"he: "<<he.faces.size()<<"/"<<he.edges.size()<<"/"<<he.vertices.size()<<std::endl;
        he.CheckBorder();
        Point3List mean_curvature_vectors;
        int iter = 0;
        auto &vertices = he.vertices;
        auto is_border = he.is_border;

        while(iter < MAX_ITERATION)
        {
            ComputeMeanCurvature(he, mean_curvature_vectors);
            for(size_t i = 0; i != vertices.size(); ++i)
            {
                if(is_border[i])
                continue;
                // update the vertices
                vertices[i].coor += lambda * mean_curvature_vectors[i]; 
                // if(mean_curvature_vectors[i].norm() > 1)
                // std::cout<<mean_curvature_vectors[i].transpose()<<std::endl;
            }
            iter ++;
        }
        TriangleMesh result;
        he.ToTriangleMesh(result);
        return std::make_shared<TriangleMesh>(result);
    }
}
}