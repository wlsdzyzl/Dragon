#include "Geometry/Structure/Voronoi2D.h"
#include "Visualization/Visualizer2D.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include <random>
#include <fstream>
/*
0 -189   77
1 -196  -93
2  -19 -103
3 -17  77
*/
using namespace dragon;
visualization::Visualizer2D visualizer(800, 800);
geometry::Voronoi2D vor;
geometry::Point2List pressed_points;
int points_num = 100;
std::random_device rd;
std::uniform_real_distribution<double> uni_real(-0.95, 0.95);
geometry::BoundingBox bb;
static void glfw_mouse_2d(GLFWwindow* window, int button, int action,
                        int mods)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
    // if (!ImGui::GetIO().WantCaptureMouse)
    // {
    //     mouse_buttons[button] = (action == GLFW_PRESS);
    // }
    if((button == GLFW_MOUSE_BUTTON_RIGHT) && (action == GLFW_RELEASE))
    {
        double x, y;
        visualization::window::CursorPos(x, y);
        pressed_points.push_back(geometry::Point2((x / visualization::window::w_width) * 2 - 1, (1 - y / visualization::window::w_height) * 2 - 1));
        visualizer.points_group[0] = pressed_points;
    }
}
void RegisterMouseAndKeyboard()
{
    // glfwSetKeyCallback(window, glfw_keyboard);
    glfwSetMouseButtonCallback(visualization::window::window, glfw_mouse_2d);
}
void RenderGuiComponents()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();        
    {
        ImGui::Begin("Menu");                         
        //ImGui::SameLine();
        // ImGui::BeginMenu("Curvature");
        
        ImGui::Text("Click the right mouse button to put point on the canvas, or");
        if(ImGui::Button("Random points"))
        {
            pressed_points.clear();
            visualizer.Reset();
            visualizer.points_group.resize(1);
#if 1
            for(int i = 0; i != points_num; ++i)
            {
                pressed_points.push_back(geometry::Point2(uni_real(rd), uni_real(rd)));
            }
#else 
            // pressed_points.push_back(visualizer.CanvasCoordinate( geometry::Point2(-189, 77)));
            // pressed_points.push_back(visualizer.CanvasCoordinate( geometry::Point2(-189, -93)));
            // pressed_points.push_back(visualizer.CanvasCoordinate( geometry::Point2(-17, -93)));
            // pressed_points.push_back(visualizer.CanvasCoordinate( geometry::Point2(-17, 77)));
            std::ifstream ifs("./error.txt");
            std::string line;
            while(getline(ifs, line))
            {
                std::istringstream iss(line);
                int id;
                double x, y;
                iss>>id>>x>>y;
                pressed_points.push_back(visualizer.CanvasCoordinate(geometry::Point2(x, y)));
            }
#endif
            // pressed_points.resize(91);
            visualizer.points_group[0] = pressed_points;

        }
        ImGui::SameLine();
        ImGui::SliderInt("", &points_num, 3, 1000);
        if(ImGui::Button("Voronoi"))
        {
            visualizer.Reset();
            visualizer.points_group.push_back(pressed_points);
            geometry::Point2List points;
            for(size_t i = 0; i != pressed_points.size(); ++i)
            {
                points.push_back(visualizer.RealCoordinate(pressed_points[i]));
                std::cout<<i<<" "<<points.back().transpose()<<std::endl;
            }
            vor.FromPoints(points);
            vor.GenerateDiagram();
            vor.BBTruncation(bb);
            visualizer.AddVoronoi(vor);
        }
        ImGui::SameLine();
        if(ImGui::Button("Voronoi Relaxation"))
        {
            visualizer.Reset();
            visualizer.points_group.push_back(pressed_points);
            geometry::Point2List points;
            for(size_t i = 0; i != pressed_points.size(); ++i)
            {
                points.push_back(visualizer.RealCoordinate(pressed_points[i]));
                // std::cout<<i<<" "<<points.back().transpose()<<std::endl;
            }
            vor.FromPoints(points);
            vor.GenerateDiagram();
            vor.BBTruncation(bb);
            geometry::Point2List new_site_points = vor.Relaxation();
            for(size_t i = 0; i != new_site_points.size(); ++i)
            {
                pressed_points[i] = visualizer.CanvasCoordinate(new_site_points[i]);
                // std::cout<<new_site_points[i]<<std::endl;
            }
            visualizer.Reset();
            visualizer.points_group.resize(1);
            visualizer.points_group[0] = pressed_points;
            vor.FromPoints(new_site_points);
            vor.GenerateDiagram();
            vor.BBTruncation(bb);
            visualizer.AddVoronoi(vor);
        }
        if(ImGui::Button("Delaunay Triangulation"))
        {
            visualizer.Reset();
            visualizer.points_group.push_back(pressed_points);
            geometry::Point2List points;
            for(size_t i = 0; i != pressed_points.size(); ++i)
            {
                points.push_back(visualizer.RealCoordinate(pressed_points[i]));
                // std::cout<<i<<" "<<points.back().transpose()<<std::endl;
            }
            vor.FromPoints(points);
            vor.GenerateDiagram();
            geometry::mesh::TriangleMesh mesh;
            vor.ToDualTriangleMesh(mesh);     
            visualizer.AddTriangleMesh(mesh);  
            // mesh.WriteToPLY("2dmesh.ply");     
        }

        // if(ImGui::Button("Line arrangement"))
        // {

        // }
        if(ImGui::Button("Reset"))
        {
            visualizer.Reset();
            pressed_points.clear();
            visualizer.points_group.push_back(geometry::Point2List());
            vor.Reset();
        }    
        ImGui::Text("Points on canvas: %d", (int)pressed_points.size());
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }
    // Rendering
    ImGui::Render();
}
int main()
{


    visualizer.Initialize();
    bb.x_min = visualizer.x_range(0) * 0.99;
    bb.x_max = visualizer.x_range(1) * 0.99;
    bb.y_min = visualizer.y_range(0) * 0.99;
    bb.y_max = visualizer.y_range(1) * 0.99;
    bb.z_min = 0;
    bb.z_max = 0;

    RegisterMouseAndKeyboard();
    visualizer.points_group.push_back(geometry::Point2List());  
    // for(size_t i = 0; i != points.size(); ++i)
    // visualizer.line_segments.push_back(visualizer.CanvasCoordinate(points[i]));
    // for(size_t i = 0; i != points.size(); ++i)
    // visualizer.points.push_back(visualizer.CanvasCoordinate(points[i]));
    while(!glfwWindowShouldClose(visualization::window::window))
    {
        RenderGuiComponents();
        //std::cout<<"fuck"<<std::endl;
        //visualizer.points = visualization::window::pressed_points;
        visualizer.ShowOnce();
    }
    return 0;
}