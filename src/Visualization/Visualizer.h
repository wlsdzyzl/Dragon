#ifndef DRAGON_VISUALIZER_H
#define DRAGON_VISUALIZER_H
#include "Shader.h"
#include "Geometry/BasicGeometry.h"
#include <memory>
#include "Window.h"


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
        ~Visualizer()
        {
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
            projection_matrix.setZero();
            const float L = +(u0) * zNear / -fu;
            const float T = +(v0) * zNear / fv;
            const float R = -(w-u0) * zNear / -fu;
            const float B = -(h-v0) * zNear / fv;   
            projection_matrix(0, 0) = 2 * zNear / (R-L);
            projection_matrix(1, 1) = 2 * zNear / (T-B);
            projection_matrix(2, 2) = -(zFar +zNear) / (zFar - zNear);
            projection_matrix(2, 0) = (R+L)/(R-L);
            projection_matrix(2, 1) = (T+B)/(T-B);
            projection_matrix(2, 3) = -1.0;
            projection_matrix(3, 2) =  -(2*zFar*zNear)/(zFar-zNear);   
        }
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
            
            model_view_matrix.block<3, 3>(0, 0) = camera_rotation_gl.cast<double>();
            model_view_matrix.block<3, 1>(0, 3) = camera_position_gl.cast<double>();
            model_view_matrix(3, 3) = 1.0;
        }
        void PreCall()
        {
            glClearColor(1.0f,1.0f, 1.0f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
        }
        void PostCall()
        {
            glfwSwapBuffers(window::window);
        }
        void ShowOnce();
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

        const std::string shader_vert = "draw_all.vert";
        //fragment shader
        const std::string shader_frag = "draw_feedback.frag";

        std::string shader_path = "../../src/Visualization/Shaders";
        bool draw_normal = false;
        bool draw_color = false;
        bool draw_color_phong = true;
        bool has_colors = true;
        bool has_normals = true;
        bool is_initialized = false;
        geometry::Matrix4 projection_matrix;
        geometry::Matrix4 model_view_matrix;
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