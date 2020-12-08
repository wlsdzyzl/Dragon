// generate minimal surface
#include "../HalfEdge.h"
#include "./Curvature.h"

namespace dragon
{
namespace geometry
{
    std::shared_ptr<TriangleMesh> MinimalSurfaceLocal(const TriangleMesh &mesh, double lambda = 0.1, int max_iteration = 500)
    {
        HalfEdge he; 
        he.FromTriangleMesh(mesh);
        std::cout<<"he: "<<he.faces.size()<<"/"<<he.edges.size()<<"/"<<he.vertices.size()<<std::endl;
        he.CheckBorder();
        ComputeLocalWeights(he);
        Point3List update_vectors;
        int iter = 0;
        auto &vertices = he.vertices;
        auto is_border = he.is_border;

        while(iter < max_iteration)
        {
            for(size_t i = 0; i != vertices.size(); ++i)
            {
                if(is_border[i])
                continue;
                auto start_edge = vertices[i].inc_edge;
                auto current_edge = start_edge->twin_edge->next_edge;
                std::vector<size_t> first_neighbors;
                double sum_weight = 0.0;
                geometry::Vector3 update_vector = geometry::Vector3::Zero();
                sum_weight += start_edge->weight;
                update_vector += start_edge->weight * (start_edge->ori_vertex->coor - start_edge->des_vertex->coor);
                while(current_edge != start_edge)
                {
                    sum_weight += current_edge->weight;
                    update_vector += current_edge->weight * (current_edge->ori_vertex->coor - current_edge->des_vertex->coor);
                    first_neighbors.push_back(current_edge->des_vertex->id);
                    current_edge = current_edge->twin_edge->next_edge;
                }
                // std::cout<<update_vector.transpose()<<std::endl;
                vertices[i].coor -= lambda * update_vector / sum_weight;
            }
            iter ++;
        }
        TriangleMesh result;
        he.ToTriangleMesh(result);
        return std::make_shared<TriangleMesh>(result);
    }
}
}