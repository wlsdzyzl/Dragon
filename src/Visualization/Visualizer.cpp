#include "Visualizer.h"
namespace dragon
{
namespace visualization
{
    bool Visualizer::Initialize()
    {
        window::Initialize(width, height);
        program = std::make_shared<Shader>();

        if(!program->Load(shader_path + "/" +shader_vert, 
            shader_path + "/" +shader_frag))
        {
            program = nullptr;
            std::cout<<RED<<"[ERROR]::[Visualizer]::Cannot load shaders."<<RESET<<std::endl;
            return false;
        }

        SetProjectionMatrix(width, height, width * 0.66, width * 0.66, width * 0.5, 
            width * 0.375, 0.1, 1000);
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
    void Visualizer::SetProjectionMatrix(int w, int h, float fu, float fv, float u0, 
        float v0, float zNear, float zFar)
    {
        // see https://wlsdzyzl.top/2018/11/14/%E5%9B%BE%E5%BD%A2%E5%AD%A6%E2%80%94%E2%80%94Viewing/
        // and http://www.songho.ca/opengl/gl_projectionmatrix.html
        window::projection_matrix.setZero();
        const float L = +(u0) * zNear / -fu;
        const float T = +(v0) * zNear / fv;
        const float R = -(w-u0) * zNear / -fu;
        const float B = -(h-v0) * zNear / fv;   
        window::projection_matrix(0, 0) = 2 * zNear / (R-L);
        window::projection_matrix(1, 1) = 2 * zNear / (T-B);
        window::projection_matrix(2, 2) = -(zFar +zNear) / (zFar - zNear);
        window::projection_matrix(2, 3) =  -(2*zFar*zNear)/(zFar-zNear); 
        window::projection_matrix(0, 2) = (R+L)/(R-L);
        window::projection_matrix(1, 2) = (T+B)/(T-B);
        window::projection_matrix(3, 2) = -1.0;
        window::near = zNear;
        window::far = zFar;
        window::fovy = 2 * atan(1.0 / window::projection_matrix(1, 1));
        window::aspect = w / h;
        window::up = tan(window::fovy / 2.0f) * window::near;
        window::right = window::aspect * window::up;

            
    }
    Visualizer::~Visualizer()
    {
        delete[] point_buffer;
        delete[] index_buffer;
        glDeleteBuffers(1, &ebo);
        glDeleteBuffers(1, &vbo);
        window::Cleanup();
    }
    void Visualizer::PostCall()
    {
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window::window);
        glFinish();
    }
    void Visualizer::SetModelViewMatrix(const geometry::TransformationMatrix &camera_pose, 
        bool reversed_model)
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
        auto camera_position_gl = - camera_rotation_gl * camera_position ;
        
        window::model_view_matrix.block<3, 3>(0, 0) = camera_rotation_gl;
        window::model_view_matrix.block<3, 1>(0, 3) = camera_position_gl;
        window::model_view_matrix(3, 3) = 1.0;
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
            {
                if(wireframe_mode)
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);  
                else
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);  
                glDrawElements(GL_TRIANGLES,index_buffer_size, GL_UNSIGNED_INT,0);
            }
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
    //      mvp     <<1.3125,          0,          0, -0.0508005,
    //      0,          0,       1.75,   -2.73389,
    //      0,     1.0002,          0,    4.98702,
    //      0,          1,          0,      5.186;

    // //    std::cout<<mvp<<std::endl;
    //     std::cout<<mvp<<std::endl;
        program->SetUniform(Uniform("MVP", mvp));
        int color_type = (draw_normal ? 1 : draw_color ? 2 : 0);
        program->SetUniform(Uniform("colorType", color_type));
        if(draw_phong_shading) 
        program->SetUniform(Uniform("phong", 1));
        else program->SetUniform(Uniform("phong", 0));
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
    void Visualizer::AddPointCloud(const geometry::PointCloud &pcd)
    {
        if(dynamic_first_view)
        {
            ChooseCameraPoseFromPoints(pcd.points);
        }
        if(point_step != 0 &&geometry_type != GeometryType::POINTCLOUD)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Geometry type is not pointcloud, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();
        }
        size_t size = pcd.GetSize();
        size_t step = 3;
        
        if(pcd.HasColors()) step += 3;
        if(pcd.HasNormals()) step += 3;
        if((step != point_step || has_normals != pcd.HasNormals() || has_colors != pcd.HasColors()) && point_step != 0)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Different types of pointcloud, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();            
        }
        for(size_t i = 0;i!=size;++i)
        {
            //position
            size_t start = 0;
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = static_cast<float>(pcd.points[i](j));

            //normal
            if(pcd.HasNormals())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = static_cast<float>(pcd.normals[i](j));
            }
            //color
            if(pcd.HasColors())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = static_cast<float>(pcd.colors[i](j));
            }

        }
        buffer_data_updated = true;
        point_step = step;
        
        point_buffer_size +=step*size;

        geometry_type = GeometryType::POINTCLOUD;

        has_colors = pcd.HasColors();

        has_normals = pcd.HasNormals();

        SetShaderPath();
        window::bb += pcd.GetBoundingBox();
        if(dynamic_first_view)
        {
            ChooseCameraPoseThroughBB(window::bb);
            // ChooseCameraPoseFromPoints(mesh.points);
            SetModelViewMatrix(camera_pose_for_view);
        }
#if DEBUG_MODE
        std::cout<<BLUE<<"[Visualizer]::[INFO]::Point Buffer Size: "<<point_buffer_size;
        std::cout<<" Points: "<<point_buffer_size/point_step<<RESET<<std::endl;
#endif
        if(point_buffer_size > MAX_BUFFER_SIZE || index_buffer_size > MAX_BUFFER_SIZE)
        std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Overflow."<<RESET<<std::endl;

    }    
    void Visualizer::AddTriangleMesh(const geometry::TriangleMesh &mesh)
    {
        
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
        window::bb += mesh.GetBoundingBox();
        if(dynamic_first_view)
        {
            ChooseCameraPoseThroughBB(window::bb);
            // ChooseCameraPoseFromPoints(mesh.points);
            SetModelViewMatrix(camera_pose_for_view);
        }
        if(point_buffer_size > MAX_BUFFER_SIZE || index_buffer_size > MAX_BUFFER_SIZE)
        std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Overflow."<<RESET<<std::endl;
        
    }
}
}