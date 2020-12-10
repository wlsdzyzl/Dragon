#include "MeshParameterization.h"
#include "Smoothing.h"
namespace dragon
{
namespace geometry
{
namespace mesh
{
    void GetOrderedBorder(HalfEdge &he, std::vector<int> &ordered_vid)
    {
        auto &edges = he.edges;
        HEEdge *start_edge = nullptr;
        for(size_t i = 0; i != edges.size(); ++i)
        {
            if(edges[i].twin_edge == nullptr)
            {
                start_edge = &edges[i];
                break;
            }
        }
        // find the start edge
        ordered_vid.clear();
        HEEdge *current_edge = start_edge;
        while(true)
        {
            ordered_vid.push_back(current_edge->ori_vertex->id);
            
            current_edge = current_edge->next_edge;
            while(current_edge->twin_edge != nullptr)
            {
                current_edge = current_edge->twin_edge->next_edge;
            }
            if(current_edge == start_edge)
            break;
        }
    }
    //map to a circle
    std::shared_ptr<TriangleMesh > MeshParameterization(const TriangleMesh &mesh, double radius)
    {
        HalfEdge he; 
        
        he.FromTriangleMesh(mesh);    
        auto &vertices = he.vertices;
        std::vector<int> ordered_vid;
        
        GetOrderedBorder(he, ordered_vid);
        
        geometry::Point3List ordered_border;
        //parameterize the border
        ordered_vid.push_back(ordered_vid[0]);
        for(size_t i = 0; i != ordered_vid.size(); ++i)
        {
            ordered_border.push_back(vertices[ordered_vid[i]].coor);
        }
        std::vector<double> t = parameterization::Foley<3>(ordered_border);
        //map border to 2D map
        for(size_t i = 0; i != t.size(); ++i)
        {
            double angle = 2 * M_PI * t[i];
            Point3 tmp_p = Point3::Zero();
            tmp_p.block<2, 1>(0, 0) = Point2(cos(angle), sin(angle));
            vertices[ordered_vid[i]].coor = tmp_p;
        }
        TriangleMesh tmp_mesh;
        he.ToTriangleMesh(tmp_mesh);
        return GlobalLaplacianSmooting(tmp_mesh, 0);
    }
}
}
}