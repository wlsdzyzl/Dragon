#include "Visualizer.h"
namespace dragon
{
namespace visualization
{
    bool Visualizer::Initialize()
    {
        program = std::make_shared<Shader>();
        if(!program->Load(shader_vert, shader_frag))
        {
            program = nullptr;
            std::cout<<RED<<"[ERROR]::[Visualizer]::Cannot load shaders."<<RESET<<std::endl;
            return false;
        }
        SetProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000);
        SetModelViewMatrix(camera_pose_for_view);
        window::Initialize();
        return true;
    }
    
    void Visualizer::ConfigProgram()
    {   
        if(!program) return;

        geometry::Matrix4 tmp_mvp = projection_matrix * model_view_matrix;
        Eigen::Matrix4f mvp = tmp_mvp.cast<float>();

        program->SetUniform(Uniform("MVP", mvp));
        int color_type = (draw_normal ? 1 : draw_color ? 2 : 0);
        if(draw_color_phong) color_type = 3;
        
        program->SetUniform(Uniform("colorType", color_type));
        float s_materialShininess = 8.0f;
        Eigen::Vector4f s_materialAmbient   = Eigen::Vector4f(0.85f, 0.85f, 0.85f, 1.0f);
        Eigen::Vector4f s_materialDiffuse   = Eigen::Vector4f(0.85f, 0.85f, 0.85f, 1.0f);
        Eigen::Vector4f s_materialSpecular  = Eigen::Vector4f(1.5f, 1.5f, 1.5f, 1.0f);
        Eigen::Vector4f s_lightAmbient 	  = Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f);
        Eigen::Vector4f s_lightDiffuse      = Eigen::Vector4f(0.6f, 0.52944f, 0.4566f, 0.6f);
        Eigen::Vector4f s_lightSpecular     = Eigen::Vector4f(0.3f, 0.3f, 0.3f, 1.0f);
        Eigen::Vector3f lightDir 	= Eigen::Vector3f(0.0f, -1.0f, 2.0f);

        program->SetUniform(Uniform("materialShininess", s_materialShininess));
        program->SetUniform(Uniform("materialAmbient", s_materialAmbient));
        program->SetUniform(Uniform("materialDiffuse", s_materialDiffuse));
        program->SetUniform(Uniform("materialSpecular", s_materialSpecular));
        program->SetUniform(Uniform("lightAmbient", s_lightAmbient));
        program->SetUniform(Uniform("lightDiffuse", s_lightDiffuse));
        program->SetUniform(Uniform("lightSpecular", s_lightSpecular));
        program->SetUniform(Uniform("lightDir", lightDir));        
    }
    void Visualizer::ShowOnce()
    {
        //std::cout<<"point_step: "<<point_step<<std::endl;
        if(buffer_data_updated)
        {
            glBindBuffer(GL_ARRAY_BUFFER,vbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, point_buffer_size * sizeof(float), &point_buffer[0]);     

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, index_buffer_size * sizeof(int), &index_buffer[0]);

            buffer_data_updated = false;
        }
        PreCall();

        // set this program as current program
        
        program->Enable();
        ConfigProgram();
        if(point_step == 0)
            std::cout<<"[Visualizer]::[WARNING]::Nothing is in the buffer"<<std::endl;
        else
        {
        
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, point_step * sizeof(float), reinterpret_cast<GLvoid*>(0));//vertex
            if(point_step > 3)
            {
                glEnableVertexAttribArray(1);
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE,point_step * sizeof(float), reinterpret_cast<GLvoid*>(3*sizeof(float)));                
            }

            if(point_step > 6)
            {
                glEnableVertexAttribArray(2);
                glVertexAttribPointer(2, 3, GL_FLOAT, GL_TRUE, point_step * sizeof(float),reinterpret_cast< GLvoid*>(6*sizeof(float)));
            }
                
            if(geometry_type == GeometryType::TRIANGLE_MESH)
                glDrawElements(GL_TRIANGLES,index_buffer_size, GL_UNSIGNED_INT,0);
            
            if(geometry_type == GeometryType::POINTCLOUD)
                glDrawArrays(GL_POINTS, 0, point_buffer_size/point_step);

            glDisableVertexAttribArray(0);
            if(point_step > 3) glDisableVertexAttribArray(1);
            if(point_step > 6) glDisableVertexAttribArray(2);
        }
        //glBindBuffer(GL_ARRAY_BUFFER, 0);

        program->Disable();
        //pangolin::glDrawAxis(3);
        PostCall();
    }
}
}