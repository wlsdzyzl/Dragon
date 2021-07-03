#ifndef DRAGON_VISUALIZER_H
#define DRAGON_VISUALIZER_H
#include "Shader.h"
#include "Geometry/BasicGeometry.h"
#include <memory>
#include "Window.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include "Geometry/Structure/BoundingBox.h"
#include "Geometry/Structure/PointCloud.h"
#include "Geometry/Structure/Octree.h"
#define MAX_BUFFER_SIZE 1024*1024*30
namespace dragon
{
namespace visualization
{
    enum GeometryType
    {
        POINTCLOUD, TRIANGLE_MESH
    };
    class Visualizer
    {

        public:
        bool Initialize();
        void ConfigProgram();
        Visualizer(int w = 800, int h = 600):width(w), height(h)
        {
            Reset();
            point_buffer = new float[MAX_BUFFER_SIZE];
            index_buffer = new int[MAX_BUFFER_SIZE];
            memset(point_buffer,0,MAX_BUFFER_SIZE*sizeof(float));
            memset(index_buffer,0,MAX_BUFFER_SIZE*sizeof(int));
            clear_color = Eigen::Vector3f(0.8, 0.8, 0.8);
            program = std::make_shared<Shader>();
            program_for_points = std::make_shared<Shader>();

        }
        ~Visualizer();
        void SetGeometryType(GeometryType type)
        {
            geometry_type = type;
        }
        void Reset()
        {
            point_step = 0;
            point_buffer_size = 0;
            index_buffer_size = 0;
            geometry_type = GeometryType::TRIANGLE_MESH;
            line_segments.clear();
            polygons.clear();
            window::bb = geometry::BoundingBox();
        }
        void SetProjectionMatrix(int w, int h, float fu, float fv, float u0, 
            float v0, float zNear, float zFar);
        void AddTriangleMesh(const geometry::TriangleMesh &mesh);
        void AddPointCloud(const geometry::PointCloud &pcd);
        void AddOctree(const geometry::Octree &oct);
        void Show();
        void ShowOnce();
        void SetModelViewMatrix(const geometry::TransformationMatrix &camera_pose, 
            bool reversed_model = false);
        void PreCall()
        {
            glfwPollEvents();
            glClearColor(clear_color(0), clear_color(1), clear_color(2),1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            window::SetProjectionMatrix();
            window::SetModelViewMatrix();
        }
        void PostCall();
        void ChooseCameraPoseFromPoints(const geometry::Point3List &points)
        {
            geometry::Point3 average_point =  geometry::Point3::Zero();
            for(size_t i = 0; i != points.size(); ++i)
            {
                average_point += points[i];
            }
            average_point /= points.size();
            //just change the y value
            geometry::TransformationMatrix camera_pose = geometry::TransformationMatrix::Identity();
            camera_pose.block<3, 1>(0, 3) = average_point + geometry::Point3(0, -5, 0);
            //set camera
            // camera_pose.block<3, 1>(0, 0) = geometry::Point3(1, 0, 0);
            camera_pose.block<3, 1>(0, 1) = geometry::Point3(0, 0, -1);
            camera_pose.block<3, 1>(0, 2) = geometry::Point3(0, 1, 0);
            camera_pose_for_view = camera_pose;
        }
        void ChooseCameraPoseThroughBB(const geometry::BoundingBox &bb)
        {
            geometry::Point3 average_point =  bb.Center();
            //just change the y value
            geometry::TransformationMatrix camera_pose = geometry::TransformationMatrix::Identity();
            double back = 2 * (bb.z_max - bb.z_min);
            if(back <= 0.0) back = 2 * (bb.x_max - bb.x_min);
            camera_pose.block<3, 1>(0, 3) = average_point - geometry::Vector3(0, 0, back);
            //set camera
            // camera_pose.block<3, 1>(0, 0) = geometry::Point3(1, 0, 0);
            // camera_pose.block<3, 1>(0, 1) = geometry::Point3(0, 0, -1);
            // camera_pose.block<3, 1>(0, 2) = geometry::Point3(0, 1, 0);
            camera_pose_for_view = camera_pose;            
            // set light source, for phong model
            light_pos = (average_point + geometry::Vector3(0, back, 0)).cast<float>();
        }
        void SetShaderPath()
        {
            if(!has_colors && !has_normals)
            shader_vert = "draw_point.vert";
            if(has_colors && !has_normals)
            shader_vert = "draw_color.vert";
            if(!has_colors && has_normals)
            shader_vert = "draw_normal.vert";
            if(has_colors && has_normals)
            shader_vert = "draw_all.vert";
            if(!program->Load(shader_path + "/" +shader_vert, 
                shader_path + "/" +shader_frag))
            {
                program = nullptr;
                std::cout<<RED<<"[ERROR]::[Visualizer]::Cannot load shaders."<<RESET<<std::endl;
            }
            if(!program_for_points->Load(shader_path + "/draw_point.vert", 
                shader_path + "/" +shader_frag))
            {
                program_for_points = nullptr;
                std::cout<<RED<<"[ERROR]::[Visualizer]::Cannot load shaders."<<RESET<<std::endl;
            }
            std::cout<<BLUE<<"[Visualizer]::[INFO]::Using shader: "<<shader_vert<<RESET<<std::endl;
        }
        size_t point_step;
        float * point_buffer;
        int * index_buffer;
        size_t point_buffer_size;
        size_t index_buffer_size;
        bool buffer_data_updated = false;

        
        //for model view
        geometry::TransformationMatrix camera_pose_for_view = geometry::TransformationMatrix::Identity();
        bool dynamic_first_view = true;
        //vertex shader

        std::string shader_vert = "draw_triangle.vert";
        //fragment shader
        const std::string shader_frag = "naive_color.frag";

        std::string shader_path = "../../src/Visualization/Shaders";
        // std::string shader_path = "/media/wlsdzyzl/wlsdzyzl_2/Dragon/src/Visualization/Shaders";
        bool draw_normal = false;
        int width;
        int height;
        bool draw_color = false;
        bool draw_phong_shading = true;
        bool has_colors = true;
        bool has_normals = true;
        bool is_initialized = false;
        bool wireframe_mode = false;
        Eigen::Vector3f clear_color;
        Eigen::Vector3f light_pos;
        std::shared_ptr<Shader> program;
        std::shared_ptr<Shader> program_for_points;
        GeometryType geometry_type;
        geometry::Point3List line_segments;
        std::vector<geometry::Point3List> polygons;
        geometry::Point3List color_table;
        protected:
        GLuint vbo;
        GLuint ebo;
        GLuint vao;
    };
}
}
#endif