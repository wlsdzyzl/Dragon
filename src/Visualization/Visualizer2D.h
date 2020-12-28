#ifndef DRAGON_VISUALIZER_2D_H
#define DRAGON_VISUALIZER_2D_H
#include "Window.h"
#include "Geometry/Structure/HalfEdge.h"
#include "Geometry/Structure/Voronoi2D.h"
#include "IO/ConsoleColor.h"
namespace dragon
{
namespace visualization
{
    class Visualizer2D
    {
        public:
        bool Initialize()
        {
            window::Initialize(width, height, "Dragon 2D");
            SetRange(-width / 2, width / 2, - height / 2, height / 2);
            for(size_t i = 0; i < io::color_table.size(); ++i)
            {
                color_table.push_back( geometry::Point3(io::color_table[i][0] / 255.0, io::color_table[i][1] / 255.0, io::color_table[i][2] / 255.0 ));
            }
            return true;
        }
        void ConfigProgram();
        Visualizer2D(int w = 800, int h = 600):width(w), height(h)
        {
            Reset();
            clear_color = geometry::Point3(0.95, 0.95, 0.95);
        }
        ~Visualizer2D()
        {
            window::Cleanup();
        }
        void Show();
        void ShowOnce();
        void Reset()
        {
            points_group.clear();
            line_segments.clear();
            polygons.clear();
            line_strip.clear();
        }
        void AddHalfEdge(const geometry::HalfEdge &he);
        void AddVoronoi(const geometry::Voronoi2D &v);
        void AddTriangleMesh(const geometry::TriangleMesh &mesh);
        void SetRange(double x_min, double x_max, double y_min, double y_max)
        {
            x_range = geometry::Point2(x_min, x_max);
            y_range = geometry::Point2(y_min, y_max);
        }
        void PreCall()
        {
            glfwPollEvents();
            glClearColor(clear_color(0), clear_color(1), clear_color(2),1.0f);
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
        std::vector<geometry::Point2List> polygons;
        geometry::Point2List line_strip;
        geometry::Point3List color_table;
        ImVec4 default_color;
        double point_radius = 0.01;
        geometry::Point2 x_range;
        geometry::Point2 y_range;
        Eigen::Vector3f clear_color;
    
    };
}
}
#endif