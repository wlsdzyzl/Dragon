#include "Geometry/Structure/Voronoi2D.h"
#include "Visualization/Visualizer2D.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include "Geometry/Curve.h"
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
int target_num = 1000;
std::random_device rd;
std::uniform_real_distribution<double> uni_real(-0.95, 0.95);
geometry::BoundingBox bb;
bool remain_convex_hull = false;
bool four_points = true;
bool bezier_interp = true;
bool cubic_spline_0 = true;
bool cubic_spline_1 = true;
bool bezier = true;
bool b_cubic_spline = true;
bool chaikin = true;

// geometry::Point2List four_points_p = visualizer.line_strips[0];
// geometry::Point2List bezier_interp_p = visualizer.line_strips[1];
// geometry::Point2List cubic_spline_0_p = visualizer.line_strips[2];
// geometry::Point2List cubic_spline_1_p = visualizer.line_strips[3];
// geometry::Point2List bezier_p = visualizer.line_strips[4];
// geometry::Point2List b_cubic_spline_p = visualizer.line_strips[5];
// geometry::Point2List chaikin_p = visualizer.line_strips[6];


void Reset()
{
    visualizer.Reset();
    visualizer.points_group.push_back(pressed_points);
    vor.Reset();
    visualizer.line_strips.resize(7);
    visualizer.line_strips[0].clear();
    visualizer.line_strips[1].clear();
    visualizer.line_strips[2].clear();
    visualizer.line_strips[3].clear();
    visualizer.line_strips[4].clear();
    visualizer.line_strips[5].clear();
    visualizer.line_strips[6].clear();
}

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
        if(ImGui::CollapsingHeader("Voronoi", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if(ImGui::Button("Generate Voronoi Diagram"))
            {
                Reset();
                geometry::Point2List points;
                for(size_t i = 0; i != pressed_points.size(); ++i)
                {
                    points.push_back(visualizer.RealCoordinate(pressed_points[i]));
                    // std::cout<<i<<" "<<points.back().transpose()<<std::endl;
                }
                vor.FromPoints(points);
                vor.GenerateDiagram();
                vor.BBTruncation(bb);
                visualizer.AddVoronoi(vor);
            }
            if(ImGui::Button("Relaxation"))
            {
                Reset();
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
                Reset();
                geometry::Point2List points;
                for(size_t i = 0; i != pressed_points.size(); ++i)
                {
                    points.push_back(visualizer.RealCoordinate(pressed_points[i]));
                    // std::cout<<i<<" "<<points.back().transpose()<<std::endl;
                }
                vor.FromPoints(points);
                vor.GenerateDiagram();
                if(!remain_convex_hull) vor.BBTruncation(bb);
                geometry::TriangleMesh mesh;
                vor.ToDualTriangleMesh(mesh);     
                visualizer.AddTriangleMesh(mesh);  
                // mesh.WriteToPLY("2dmesh.ply");     
            } 
            ImGui::SameLine();
            ImGui::Checkbox("Convex Hull", &remain_convex_hull);
        }
        if(ImGui::CollapsingHeader("Curve", ImGuiTreeNodeFlags_None))
        {
            ImGui::SliderInt("Target Number", &target_num, 500, 50000);
            ImGui::Text("Interplotation");
            ImGui::Checkbox("Four Points", &four_points);
            ImGui::ColorEdit3("color of Four Points", visualizer.color_table[1].data());
            ImGui::Checkbox("Cubic (Natural)", &cubic_spline_0);
            ImGui::ColorEdit3("color of Cubic (Natural)", visualizer.color_table[2].data());
            ImGui::Checkbox("Cubic (End Slope)", &cubic_spline_1);
            ImGui::ColorEdit3("color of Cubic (End Slope)", visualizer.color_table[3].data());
            ImGui::Checkbox("Bezier Interplotation", &bezier_interp);        
            ImGui::ColorEdit3("color of Bezier Interplotation", visualizer.color_table[4].data());
            ImGui::Text("Approximation");
            ImGui::Checkbox("Chaikin", &chaikin);  
            ImGui::ColorEdit3("color of Chaikin", visualizer.color_table[5].data());
            ImGui::Checkbox("Bezier", &bezier);
            ImGui::ColorEdit3("color of Bezier", visualizer.color_table[6].data());
            ImGui::Checkbox("BSpline (3)", &b_cubic_spline);
            ImGui::ColorEdit3("color of BSpline (3)", visualizer.color_table[7].data());
            if(ImGui::Button("Compute Curve"))
            {
                Reset();
                if(pressed_points.size() < 4)
                {
                    std::cout<<"Number of points is less than 4."<<std::endl;
                }
                else
                {
                    geometry::Point2List points, result_p;

                    for(size_t i = 0; i != pressed_points.size(); ++i)
                    {
                        points.push_back(visualizer.RealCoordinate(pressed_points[i]));
                        // std::cout<<i<<" "<<points.back().transpose()<<std::endl;
                    }

                    if(four_points)
                    {
                        result_p = geometry::curve::FourPointsInterpolation(points, target_num);
                        visualizer.line_strips[0].resize(result_p.size());
                        for(size_t i = 0; i != result_p.size(); ++i)
                        {
                            visualizer.line_strips[0][i] = visualizer.CanvasCoordinate(result_p[i]);
                        }
                    }
                    if(cubic_spline_0)
                    {
                        result_p = geometry::curve::CubicSpline(points, target_num, 0);
                        visualizer.line_strips[1].resize(result_p.size());
                        for(size_t i = 0; i != result_p.size(); ++i)
                        {
                            visualizer.line_strips[1][i] = visualizer.CanvasCoordinate(result_p[i]);
                        }
                    }
                    if(cubic_spline_1)
                    {
                        result_p = geometry::curve::CubicSpline(points, target_num, 1);
                        visualizer.line_strips[2].resize(result_p.size());
                        for(size_t i = 0; i != result_p.size(); ++i)
                        {
                            visualizer.line_strips[2][i] = visualizer.CanvasCoordinate(result_p[i]);
                        }  
                    }
                    if(bezier_interp)
                    {
                        result_p = geometry::curve::BezierInterpolation(points, target_num);
                        visualizer.line_strips[3].resize(result_p.size());
                        for(size_t i = 0; i != result_p.size(); ++i)
                        {
                            visualizer.line_strips[3][i] = visualizer.CanvasCoordinate(result_p[i]);
                        }
                    }
                    if(chaikin)
                    {
                        result_p = geometry::curve::Chaikin(points, target_num);
                        visualizer.line_strips[4].resize(result_p.size());
                        for(size_t i = 0; i != result_p.size(); ++i)
                        {
                            visualizer.line_strips[4][i] = visualizer.CanvasCoordinate(result_p[i]);
                        }
                    }    


                    if(bezier)
                    {
                        result_p = geometry::curve::Bezier(points, target_num);
                        visualizer.line_strips[5].resize(result_p.size());
                        for(size_t i = 0; i != result_p.size(); ++i)
                        {
                            visualizer.line_strips[5][i] = visualizer.CanvasCoordinate(result_p[i]);
                        }
                    }       

                    if(b_cubic_spline)
                    {
                        result_p = geometry::curve::BCubicSpline(points, target_num);
                        visualizer.line_strips[6].resize(result_p.size());
                        for(size_t i = 0; i != result_p.size(); ++i)
                        {
                            visualizer.line_strips[6][i] = visualizer.CanvasCoordinate(result_p[i]);
                        }
                    }      
                }
            }   
        }
        if(ImGui::Button("Reset"))
        {
            pressed_points.clear();
            Reset();
        }    
        ImGui::Text("Points on canvas: %d", (int)pressed_points.size());
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::ColorEdit3("Background color", visualizer.clear_color.data());
        ImGui::End();
    }
    // Rendering
    ImGui::Render();
}
int main()
{


    visualizer.Initialize();
    Reset();
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

        visualizer.ShowOnce();
    }
    return 0;
}