#ifndef DRAGON_VISUALIZER_2D_H
#define DRAGON_VISUALIZER_2D_H
#include "Window.h"
#include "Geometry/Structure/HalfEdge.h"
#include "Geometry/Structure/Voronoi2D.h"
namespace dragon
{
namespace visualization
{
    class Visualizer2D
    {
        public:
        bool Initialize()
        {
            window::Initialize(width, height, false);
            SetRange(-width / 2, width / 2, - height / 2, height / 2);
            return true;
        }
        void ConfigProgram();
        Visualizer2D(int w = 800, int h = 600):width(w), height(h)
        {
            Reset();
        }
        ~Visualizer2D()
        {
            window::Cleanup();
        }
        void Show();
        void ShowOnce();
        void Reset(){}
        void AddHalfEdge(const geometry::HalfEdge &he);
        void AddVoronoi(const geometry::Voronoi2D &v);
        void SetRange(double x_min, double x_max, double y_min, double y_max)
        {
            x_range = geometry::Point2(x_min, x_max);
            y_range = geometry::Point2(y_min, y_max);
        }
        void PreCall()
        {
            glfwPollEvents();
            glClearColor(1.0f,1.0f, 1.0f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
        }
        void PostCall()
        {
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window::window);
            glFinish();
        }
        geometry::Point2 CanvasCoordinate(const geometry::Point2 &p) const;
        geometry::Point2 RealCoordinate(const geometry::Point2 &p) const;
        int width;
        int height;
        std::vector<std::tuple<geometry::Point2, double>> circles;
        geometry::Point2List line_segments;
        std::vector<geometry::Point2List> points_group;
        geometry::Point2List polygon;
        geometry::Point2List line_strip;
        geometry::Point3List points_group_colors;
        ImVec4 default_color;
        double point_radius = 0.01;
        geometry::Point2 x_range;
        geometry::Point2 y_range;
    
    };
}
}
#endif