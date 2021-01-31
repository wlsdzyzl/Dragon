// Draw spider man
#include "Visualization/Visualizer2D.h"
#include <random>
#include <fstream>
#include "Geometry/Curve.h"
#include <unistd.h>
using namespace dragon;
using namespace visualization;
using namespace window;
using namespace geometry;
visualization::Visualizer2D visualizer(800, 800);
void CRenderGuiComponents()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();        
    ImGui::Render();
}
void PreCall()
{
    glfwPollEvents();
    glClearColor(1.0, 1.0, 1.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
void PostCall()
{
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window::window);
    glFinish();
}
int main()
{

    Initialize();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    
    while(!glfwWindowShouldClose(visualization::window::window))
    {
        // while(true)
        // {
        //     if(getchar() == 'c') break;
        // }
        PreCall();
        CRenderGuiComponents();
        double r = 0.6;
        
        Point2List border_points;
        int n = 40;
        for(int i = 0; i < n; ++i)
        {
            border_points.push_back(Point2(cos((i + 0.0f) / n * 2 * M_PI) * r,  sin((i + 0.0f) / n * 2 * M_PI) * r));
        }   
        for(int i = 1; i < n; ++i)
        {
            for(int j = 0; j < i; ++j)
            {
                DrawLine(border_points[i], border_points[j], Eigen::Vector3f(1, 0, 0), 6);
                PostCall();
            }
        }  
        int group = 17;
        int inter_group = 9;
        Point2List nodes;
        Point2List first_nodes;
        for(int i = 0; i != inter_group; ++i)
        {
            double tmp_r = (i+1.0) / inter_group * r;
            nodes.clear();
            for(int j = 0; j != group; ++j)
            {
                nodes.push_back(Point2(cos((j + 0.0f) / group * 2 * M_PI) * tmp_r,  sin((j + 0.0f) / group * 2 * M_PI) * tmp_r));
            }
            if(i != inter_group - 1 && i != 0)
            DrawPolygon(nodes,  Eigen::Vector3f(0, 0, 0), 4); PostCall();
            if(i == 0)
            first_nodes = nodes;
        
        }
        
        for(int i = 0; i != group; ++i)
        {
            first_nodes[i] /= 1.5;
            DrawLine(nodes[i], first_nodes[i], Eigen::Vector3f(0, 0, 0), 4); PostCall();
        }
        DrawPolygon(first_nodes,  Eigen::Vector3f(0, 0, 0), 4); PostCall();
        {
            Point2 lp0(-0.03, -0.05), lp1(-0.44, 0.33),lp2(-0.15, -0.3), lp3(-0.33, -0.3), lp4(-0.5, 0);
            Point2List inter_points = {lp0, lp2, lp3, lp4, lp1};
            // auto generate_points = curve::CubicSpline(inter_points, 500, 1); 
            auto generate_points = curve::Chaikin(inter_points, 500); 
            generate_points.push_back(lp1);
            for(size_t i = 1; i != generate_points.size() - 1; ++i)
            {
                Point2List tmp_p = {lp0, generate_points[i], generate_points[i+1]};
                DrawPolygonFilled(tmp_p, Eigen::Vector3f(0.9, 0.9, 0.9));PostCall();
            }
            DrawLineStrip(generate_points,  Eigen::Vector3f(0, 0, 0), 6);PostCall();
            DrawLine(lp0, lp1, Eigen::Vector3f(0, 0, 0), 6); PostCall();
        }
        {
            Point2 rp0(0.03, -0.05), rp1(0.44, 0.33), rp2(0.15, -0.3), rp3(0.33, -0.3), rp4(0.5, 0);
            Point2List inter_points = {rp0, rp2, rp3, rp4, rp1};
            auto generate_points = curve::CubicSpline(inter_points, 500, 1); 
            generate_points.push_back(rp1);
            for(size_t i = 1; i != generate_points.size() - 1; ++i)
            {
                Point2List tmp_p = {rp0, generate_points[i], generate_points[i+1]};
                DrawPolygonFilled(tmp_p, Eigen::Vector3f(0.9, 0.9, 0.9));PostCall();
            }
            DrawLineStrip(generate_points,  Eigen::Vector3f(0, 0, 0), 6);PostCall();
            DrawLine(rp0, rp1, Eigen::Vector3f(0, 0, 0), 6); PostCall();
        }
        DrawCircle(Point2(0, 0), r, Eigen::Vector3f(0, 0, 0), 6, 1000); PostCall();
        sleep(1);
    }
    while(true) ;
    return 0;
}