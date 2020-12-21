#include "Geometry/Structure/Voronoi2D.h"
#include "Visualization/Visualizer2D.h"
using namespace dragon;
int main()
{
    geometry::Voronoi2D vor;
    geometry::Point2List points;
    visualization::Visualizer2D visualizer(800, 800);
    points.push_back(geometry::Point2(0, 0));
    points.push_back(geometry::Point2(100, 200));
    points.push_back(geometry::Point2(200, 100));
    points.push_back(geometry::Point2(50, 50));
    points.push_back(geometry::Point2(245, -10));
    points.push_back(geometry::Point2(-65, 165));
    visualizer.Initialize();
    vor.FromPoints(points);
    vor.GenerateDiagram();
    visualizer.AddVoronoi(vor);
    
    // for(size_t i = 0; i != points.size(); ++i)
    // visualizer.line_segments.push_back(visualizer.CanvasCoordinate(points[i]));
    // for(size_t i = 0; i != points.size(); ++i)
    // visualizer.points.push_back(visualizer.CanvasCoordinate(points[i]));
    while(!glfwWindowShouldClose(visualization::window::window))
    {
        visualization::window::RenderGuiComponents();
        //std::cout<<"fuck"<<std::endl;
        //visualizer.points = visualization::window::pressed_points;
        visualizer.ShowOnce();
    }
    return 0;
}