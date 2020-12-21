#include "Visualizer2D.h"
namespace dragon
{
namespace visualization
{
    void Visualizer2D::Show()
    {
        Initialize();

        while(!glfwWindowShouldClose(window::window))
        {
            window::RenderGuiComponents();
            ShowOnce();
        }
    }
    geometry::Point2 Visualizer2D::CanvasCoordinate(const geometry::Point2 &p) const
    {
        geometry::Point2 res;
        // std::cout<<p.transpose()<<std::endl;
        res(0) = -1 + (p(0) - x_range(0))/(x_range(1) - x_range(0)) * 2;
        res(1) = -1 + (p(1) - y_range(0))/(y_range(1) - y_range(0)) * 2;
        // std::cout<<res.transpose()<<std::endl;
        // std::cout<<res.transpose()<<std::endl;
        return res;
    }
    geometry::Point2 Visualizer2D::RealCoordinate(const geometry::Point2 &p) const
    {
        geometry::Point2 res;
        res(0) = 0.5 * (p(0) + 1) * (x_range(1) - x_range(0)) - x_range(0) ;
        res(1) = 0.5 * (p(1) + 1) * (y_range(1) - y_range(0)) - y_range(0);
        return res;
    }
    void Visualizer2D::ShowOnce()
    {

        PreCall();
        for(size_t gid = 0; gid != points_group.size(); ++gid)
        {
            auto &points = points_group[gid];
            window::DrawPoints(points, points_group_colors[gid]);
        }
        window::DrawLines(line_segments);

        for(size_t i = 0; i != circles.size(); ++i)
        {
            geometry::Point2 o;
            double r;
            std::tie(o, r) = circles[i];
            window::DrawCircle(o, r);
        } 
        window::DrawPolygon(polygon);
        PostCall();
    }
    void Visualizer2D::AddHalfEdge(const geometry::HalfEdge &he)
    {
        auto &edges = he.edges;
        auto &vertices = he.vertices;
        auto &face = he.faces;
        for(size_t i = 0; i != edges.size(); ++i)
        {
            if(edges[i].ori_vertex && edges[i].des_vertex)
            {
                line_segments.push_back(CanvasCoordinate(edges[i].ori_vertex->coor.head<2>()));
                line_segments.push_back(CanvasCoordinate(edges[i].des_vertex->coor.head<2>()));
                //std::cout<<"!!"<<std::endl;
            }
            else if(!edges[i].ori_vertex && !edges[i].des_vertex)
            {
                std::cout<<YELLOW<<"[WARNING]::Edge "<<edges[i].id<<"'s two end vertices are nullptr, which shouldn't occur."<<RESET<<std::endl;
            }
        }
        geometry::Point2List points;
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            points.push_back(CanvasCoordinate(vertices[i].coor.head<2>()));
            //std::cout<<"fuck: "<<CanvasCoordinate(vertices[i].coor.head<2>())<<std::endl;
        }
        for(size_t i = 0; i != face.size(); ++i)
        {

        }
        points_group.push_back(points);
        points_group_colors.push_back(geometry::Point3(1, 0, 0)); 
    }
    void Visualizer2D::AddVoronoi(const geometry::Voronoi2D &v)
    {
        AddHalfEdge(v.he);
        geometry::Point2List points;
        for(size_t i = 0; i != v.site_points.size(); ++i)
        {
            points.push_back(CanvasCoordinate(v.site_points[i]));
        }
        points_group.push_back(points);
        points_group_colors.push_back(geometry::Point3(0, 1, 0)); 
        // draw bb
        geometry::Point2 left_top(v.bb.x_min, v.bb.y_max), left_bottom(v.bb.x_min, v.bb.y_min);
        geometry::Point2 right_bottom(v.bb.x_max, v.bb.y_min), right_top(v.bb.x_max, v.bb.y_max);
        polygon.push_back(CanvasCoordinate(left_top));
        polygon.push_back(CanvasCoordinate(left_bottom));
        polygon.push_back(CanvasCoordinate(right_bottom));
        polygon.push_back(CanvasCoordinate(right_top));
    }
}
}