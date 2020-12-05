#ifndef DRAGON_VISUALIZER_H
#define DRAGON_VISUALIZER_H
#include "Shader.h"
#include "Geometry/BasicGeometry.h"
#include <memory>
#include "Window.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"
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
        Visualizer()
        {
            Reset();
            point_buffer = new float[MAX_BUFFER_SIZE];
            index_buffer = new int[MAX_BUFFER_SIZE];
            memset(point_buffer,0,MAX_BUFFER_SIZE*sizeof(float));
            memset(index_buffer,0,MAX_BUFFER_SIZE*sizeof(int));

        }
        ~Visualizer()
        {
            delete[] point_buffer;
            delete[] index_buffer;
            glDeleteBuffers(1, &ebo);
            glDeleteBuffers(1, &vbo);
            window::Cleanup();
        }
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
        }
        void SetProjectionMatrix(int w, int h, float fu, float fv, float u0, 
            float v0, float zNear, float zFar)
        {
            // see https://wlsdzyzl.top/2018/11/14/%E5%9B%BE%E5%BD%A2%E5%AD%A6%E2%80%94%E2%80%94Viewing/
            window::projection_matrix.setZero();
            const float L = +(u0) * zNear / -fu;
            const float T = +(v0) * zNear / fv;
            const float R = -(w-u0) * zNear / -fu;
            const float B = -(h-v0) * zNear / fv;   
            window::projection_matrix(0, 0) = 2 * zNear / (R-L);
            window::projection_matrix(1, 1) = 2 * zNear / (T-B);
            window::projection_matrix(2, 2) = -(zFar +zNear) / (zFar - zNear);
            window::projection_matrix(2, 3) =  -(2*zFar*zNear)/(zFar-zNear); 
            // window::projection_matrix(0, 2) = (R+L)/(R-L);
            // window::projection_matrix(1, 2) = (T+B)/(T-B);
            window::projection_matrix(3, 2) = -1.0;
              
        }
        void AddTriangleMesh(const geometry::TriangleMesh &mesh);
        void Show();
        void ShowOnce();
        void SetModelViewMatrix(const geometry::TransformationMatrix &camera_pose, 
            bool reversed_model = true)
        {
            //transform to camera coordinate system
            geometry::Matrix3 camera_rotation = camera_pose.block<3, 3>(0, 0);
            
            geometry::Vector3 camera_position = camera_pose.block<3, 1>(0, 3);
            //in OpenGL
            //z
            geometry::Vector3 forward_gl = - camera_rotation.block<3, 1>(0, 2);
            //y
            geometry::Vector3 up_gl = camera_rotation.block<3, 1>(0, 1);
            if(reversed_model) up_gl = -up_gl;
            //x
            geometry::Vector3 right_gl = camera_rotation.block<3, 1>(0, 0);
            up_gl.normalize();
            right_gl.normalize();
            forward_gl.normalize();
            
            geometry::Matrix3 camera_rotation_gl;
            // axis of OpenGL coordinate system
            camera_rotation_gl.block<1, 3>(0, 0) = right_gl.transpose();
        
            camera_rotation_gl.block<1, 3>(1, 0) = up_gl.transpose();

            camera_rotation_gl.block<1, 3>(2, 0) = forward_gl.transpose();
            // change the zero point
            // we want the camera can be more far away to the object
            // because in OpenGL coordinate, the Z axis is towards the camera, so if camera wants to move far away from the 
            // objects, it need to add the z axis vector.
            auto camera_position_gl = - camera_rotation_gl * camera_position + 2 * forward_gl;
            
            window::model_view_matrix.block<3, 3>(0, 0) = camera_rotation_gl;
            window::model_view_matrix.block<3, 1>(0, 3) = camera_position_gl;
            window::model_view_matrix(3, 3) = 1.0;
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
        }
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
        bool draw_normal = false;
        bool draw_color = false;
        bool draw_color_phong = true;
        bool has_colors = true;
        bool has_normals = true;
        bool is_initialized = false;

        std::shared_ptr<Shader> program;
        GeometryType geometry_type;
        protected:
        GLuint vbo;
        GLuint ebo;
        GLuint vao;
    };
}
}
#endif