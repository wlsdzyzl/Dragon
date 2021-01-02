#include "MeshParameterization.h"
#include "Smoothing.h"
#include "Curvature.h"
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
            if(edges[i]->twin_edge == nullptr)
            {
                start_edge = edges[i];
                break;
            }
        }
        // find the start edge
        if(start_edge == nullptr)
            return;
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
    std::shared_ptr<TriangleMesh > MeshParameterizationCircle(const TriangleMesh &mesh, int para_type, double radius)
    {
        HalfEdge he; 
     
        he.FromTriangleMesh(mesh);  
        ComputeCotanWeight(he);  
        he.CheckBorder();
        auto &vertices = he.vertices;
        std::vector<int> ordered_vid;
       
        GetOrderedBorder(he, ordered_vid);
        if(ordered_vid.size() < 3)
        {
            std::cout<<YELLOW<<"[WARNING]::[Parameterization]::The number of border points is less than 3."<<RESET<<std::endl;
            return std::make_shared<TriangleMesh>(mesh);
        }
        geometry::Point3List ordered_border;
        //parameterize the border
        ordered_vid.push_back(ordered_vid[0]);
        for(size_t i = 0; i != ordered_vid.size(); ++i)
        {
            ordered_border.push_back(vertices[ordered_vid[i]]->coor);
        }
        geometry::ScalarList t;
        if(para_type == 0)
        t = parameterization::Uniform<3>(ordered_border);
        else if(para_type == 1)
        t = parameterization::Chordal<3>(ordered_border);
        else if(para_type == 2)
        t = parameterization::Centripetal<3>(ordered_border);
        else if(para_type == 3)
        t = parameterization::Foley<3>(ordered_border);
        //map border to 2D map
        for(size_t i = 0; i != t.size(); ++i)
        {
            double angle = 2 * M_PI * t[i];
            Point3 tmp_p = Point3::Zero();
            tmp_p.block<2, 1>(0, 0) = Point2(radius * cos(angle), radius * sin(angle));
            vertices[ordered_vid[i]]->coor = tmp_p;
        }
        //std::cout<<he.has_colors<<std::endl;
        TriangleMesh tmp_mesh;
        // he.ToTriangleMesh(tmp_mesh); 
        //tmp_mesh.WriteToPLY("changed_border.ply");
        GlobalLaplacianSmooting(he, 0);
        he.ToTriangleMesh(tmp_mesh); 
        return std::make_shared<TriangleMesh>(tmp_mesh);
    }
    std::shared_ptr<TriangleMesh > MeshParameterizationSquare(const TriangleMesh &mesh, int para_type, double len)
    {
        HalfEdge he; 
     
        he.FromTriangleMesh(mesh);  
        ComputeCotanWeight(he);  
        he.CheckBorder();
        auto &vertices = he.vertices;
        std::vector<int> ordered_vid;
       
        GetOrderedBorder(he, ordered_vid);
        if(ordered_vid.size() < 3)
        {
            std::cout<<YELLOW<<"[WARNING]::[Parameterization]::The number of border points is less than 3."<<RESET<<std::endl;
            return std::make_shared<TriangleMesh>(mesh);
        }        
        geometry::Point3List ordered_border;
        //parameterize the border
        ordered_vid.push_back(ordered_vid[0]);
        for(size_t i = 0; i != ordered_vid.size(); ++i)
        {
            ordered_border.push_back(vertices[ordered_vid[i]]->coor);
        }
        geometry::ScalarList t;
        if(para_type == 0)
        t = parameterization::Uniform<3>(ordered_border);
        else if(para_type == 1)
        t = parameterization::Chordal<3>(ordered_border);
        else if(para_type == 2)
        t = parameterization::Centripetal<3>(ordered_border);
        else if(para_type == 3)
        t = parameterization::Foley<3>(ordered_border);
        //map border to 2D map
        for(size_t i = 1; i < t.size()-1; ++i)
        {
            if(t[i] >= 0.25 && t[i-1] <= 0.25)
            {
                if(0.25 - t[i-1] < t[i] - 0.25)
                t[i-1] = 0.25;
                else t[i] = 0.25;
            }
            if(t[i] >= 0.5 && t[i-1] <= 0.5)
            {
                if(0.5 - t[i-1] < t[i] - 0.5)
                t[i-1] = 0.5;
                else t[i] = 0.5;
            }
            if(t[i] >= 0.75 && t[i-1] <= 0.75)
            {
                if(0.75 - t[i-1] < t[i] - 0.75)
                t[i-1] = 0.75;
                else t[i] = 0.75;
            }
            
        }
        for(size_t i = 0; i != t.size(); ++i)
        {
            geometry::Point3 tmp;
            if(t[i] <= 0.25)
            {
                tmp = geometry::Point3(-len, len - t[i] * (8 * len), 0 );
            }
            else if(t[i] <= 0.5)
            {

                tmp = geometry::Point3((t[i] - 0.25) * (8 * len) - len, -len, 0 );
            }
            else if(t[i] <= 0.75)
            {
                tmp = geometry::Point3(len, (t[i] - 0.5) * (8 * len) - len, 0 );
            }
            else
            {
                tmp = geometry::Point3(len - (t[i] - 0.75) * (8 * len) , len, 0 );
            }

            vertices[ordered_vid[i]]->coor = tmp;
        }
        //std::cout<<he.has_colors<<std::endl;
        GlobalLaplacianSmooting(he, 0);
        TriangleMesh tmp_mesh;
        he.ToTriangleMesh(tmp_mesh); 
        return std::make_shared<TriangleMesh>(tmp_mesh);
    }
}
}
}