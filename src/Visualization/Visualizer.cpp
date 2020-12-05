#include "Visualizer.h"
namespace dragon
{
namespace visualization
{
    bool Visualizer::Initialize()
    {
        window::Initialize(800, 600);
        program = std::make_shared<Shader>();

        if(!program->Load(shader_path + "/" +shader_vert, 
            shader_path + "/" +shader_frag))
        {
            program = nullptr;
            std::cout<<RED<<"[ERROR]::[Visualizer]::Cannot load shaders."<<RESET<<std::endl;
            return false;
        }

        SetProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000);
        SetModelViewMatrix(camera_pose_for_view);
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER,vbo);
        glBufferData(GL_ARRAY_BUFFER, MAX_BUFFER_SIZE * sizeof(float), &point_buffer[0], GL_DYNAMIC_DRAW);     

        glGenBuffers(1, &ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,MAX_BUFFER_SIZE *sizeof(int), &index_buffer[0], GL_DYNAMIC_DRAW);

        std::cout<<GREEN<<"[Visualizer]::[INFO]::Initialize successfully."<<RESET << std::endl;

        is_initialized = true;
        buffer_data_updated = false;
        return true;
    }
    void Visualizer::ShowOnce()
    {
        
        if(buffer_data_updated)
        {
            glBindBuffer(GL_ARRAY_BUFFER,vbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, point_buffer_size * sizeof(float), &point_buffer[0]);     

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, index_buffer_size * sizeof(int), &index_buffer[0]);

            buffer_data_updated = false;
        }
        PreCall();
        program->Enable();
        ConfigProgram();
        
        if(point_step == 0)
        {
            GLfloat vertices[] =
            {
                -0.5f, -0.5f, 0.0f, // Left
                1.0f, 0.0f, 0.0f,
                0.5f, -0.5f, 0.0f, // Right
                0.0f, 1.0f, 0.0f,
                0.0f,  0.5f, 0.0f,  // Top
                0.0f, 0.0f, 1.0f
            };
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Nothing is in the buffer"<<RESET<<std::endl;
            // glGenVertexArrays( 1, &vao );
            // glGenBuffers( 1, &vbo );
            // Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).

            glBindBuffer( GL_ARRAY_BUFFER, vbo );
            glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
            
            glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof( GLfloat ), ( GLvoid * ) 0 );
            
            glEnableVertexAttribArray( 0 );

            glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof( GLfloat ), ( GLvoid * ) (3 * sizeof(GLfloat)) );
            
            glEnableVertexAttribArray( 1 );            
            
            glBindBuffer( GL_ARRAY_BUFFER, 0 ); // Note that this is allowed, the call to glVertexAttribPointer registered vbo as the currently bound vertex buffer object so afterwards we can safely unbind
            glDrawArrays( GL_TRIANGLES, 0, 3 );         
        }
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
    void Visualizer::Show()
    {
        if(point_step == 0)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Nothing is in the buffer, show triangle"<<RESET<<std::endl;
        }
        Initialize();

        while(!glfwWindowShouldClose(window::window))
        {
            window::RenderGuiComponents();
            ShowOnce();
        }
        
    }    
    void Visualizer::ConfigProgram()
    {   
        if(!program) return;
        if(point_step == 0) return;
        geometry::Matrix4 tmp_mvp = window::projection_matrix * window::model_view_matrix;
        Eigen::Matrix4f mvp = tmp_mvp.cast<float>();
    //     mvp <<   1.3125,        0,        0, -5.12902,
    //    0  ,      0  ,   1.75, -5.47029,
    //    0,   1.0002,        0,  2.25604,
    //    0,        1,        0,  2.45557;
    //    std::cout<<mvp<<std::endl;
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

    void Visualizer::AddTriangleMesh(const geometry::TriangleMesh &mesh)
    {
        if(dynamic_first_view)
        {
            ChooseCameraPoseFromPoints(mesh.points);
            SetModelViewMatrix(camera_pose_for_view);
            
        }
        
        if( point_step != 0 &&geometry_type != GeometryType::TRIANGLE_MESH)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Geometry type is not mesh, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();
        }
        size_t point_size = mesh.GetPointSize();
        size_t triangle_size = mesh.GetTriangleSize();
        size_t step = 3;
        if((step != point_step || has_normals != mesh.HasNormals() || has_colors != mesh.HasColors()) && point_step != 0)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Different types of mesh, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();            
        }        
        if(mesh.HasColors()) step += 3;
        if(mesh.HasNormals()) step +=3;

        for(size_t i = 0;i!=point_size;++i)
        {
            //position
            size_t start = 0;
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = static_cast<float>(mesh.points[i](j));

            //normal
            if(mesh.HasNormals())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = static_cast<float>(mesh.normals[i](j));
            }
            //color
            if(mesh.HasColors())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = static_cast<float>(mesh.colors[i](j));
            }

        }

        for(size_t i = 0; i!=triangle_size; ++i)
        {
            for(size_t j = 0;j<3; ++j)
                index_buffer[i*3 + j] = mesh.triangles[i](j);
            //std::cout <<mesh.triangles[i]<<std::endl;
        }
        buffer_data_updated = true;
        point_step = step;
        point_buffer_size += step * point_size;
        index_buffer_size += 3 * triangle_size;  

        geometry_type = GeometryType::TRIANGLE_MESH;      
        has_colors = mesh.HasColors();
        has_normals = mesh.HasNormals();
        
        SetShaderPath();
#if DEBUG_MODE
        std::cout<<BLUE<<"[Visualizer]::[INFO]::Point Buffer Size: "<<point_buffer_size;
        std::cout<<" Points: "<<point_buffer_size/point_step<<std::endl;
        std::cout<<"[Visualizer]::[INFO]::Index Buffer Size: "<<index_buffer_size;
        std::cout<<" Triangles: "<<index_buffer_size/3<<RESET<<std::endl;
#endif

        if(point_buffer_size > MAX_BUFFER_SIZE || index_buffer_size > MAX_BUFFER_SIZE)
        std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Overflow."<<RESET<<std::endl;
        
    }
}
}