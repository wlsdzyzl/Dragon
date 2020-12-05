#include "Smoothing.h"
namespace dragon
{
namespace geometry
{
    // Compute new position for each vertex
    void LaplacianSmooting(const HalfEdge &he, Point3List &new_positions)
    {
        new_positions.clear();
        auto &vertices = he.vertices;
        //auto &edges = he.edges;
        auto &is_border = he.is_border;
        if(is_border.size() == 0)
        {
            std::cout<<RED<< "[ERROR]::[LaplacianSmooting]::You need to call CheckBorder firstly."<<RESET<<std::endl;
            return;
        }
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            //compute mean Curvature
            // first step, find all 1-neighbor
            geometry::Vector3 new_position(0.0, 0.0, 0.0);
            if(is_border[i])
            {
                new_positions.push_back(new_position);
                continue;
            }
            auto start_edge = vertices[i].inc_edge;
            auto current_edge = start_edge->twin_edge->next_edge;
            std::vector<size_t> first_neighbors;

            first_neighbors.push_back(start_edge->des_vertex->id);
            while(current_edge != start_edge)
            {
                first_neighbors.push_back(current_edge->des_vertex->id);
                current_edge = current_edge->twin_edge->next_edge;
            }

            for(size_t id = 0; id != first_neighbors.size(); ++id)
            {
                new_position += vertices[first_neighbors[id]].coor;
            }

            new_positions.push_back( new_position / first_neighbors.size());
        }
    }
    std::shared_ptr<TriangleMesh> LaplacianSmooting(const TriangleMesh &mesh, int max_iteration)
    {
        HalfEdge he; 
        he.FromTriangleMesh(mesh);
        std::cout<<"he: "<<he.faces.size()<<"/"<<he.edges.size()<<"/"<<he.vertices.size()<<std::endl;
        he.CheckBorder();
        Point3List new_positions;
        int iter = 0;
        auto &vertices = he.vertices;
        auto is_border = he.is_border;

        while(iter < max_iteration)
        {
            LaplacianSmooting(he, new_positions);
            for(size_t i = 0; i != vertices.size(); ++i)
            {
                if(is_border[i])
                continue;
                // update the vertices
                vertices[i].coor = new_positions[i]; 
                // if(new_positions[i].norm() > 1)
                // std::cout<<new_positions[i].transpose()<<std::endl;
            }
            iter ++;
        }
        TriangleMesh result;
        he.ToTriangleMesh(result);
        return std::make_shared<TriangleMesh>(result);
    }
}
}