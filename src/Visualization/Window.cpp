#include "Window.h"
namespace dragon
{
namespace visualization
{
namespace window
{
    GLFWwindow* window;
    geometry::Matrix4 projection_matrix;
    geometry::Matrix4 model_view_matrix; 
    bool show_demo_window = false;
    bool show_another_window = false;
    geometry::Vector4 clear_color = geometry::Vector4(0.45f, 0.55f, 0.60f, 1.00f);   
    int w_width;
    int w_height;
    double last_x = 0;
    double last_y = 0;
    geometry::BoundingBox bb;
    double near, far, fovy;
    double aspect;
    double up;
    double right;
    bool last_point_valid;
    geometry::Point3 last_point_3d;
    // geometry::Point2List pressed_points;
    static void glfw_error_callback(int error, const char* description)
    {
        std::cout<<RED<<"[ERROR]::[Window]::Glfw Error: "<<description<<RESET<<std::endl;
    }  

    //callback function when we move mouse and keyboard
    static void glfw_motion(GLFWwindow* window, double xpos, double ypos)
    {
        Movement(xpos, ypos);
    }
    static void glfw_scroll(GLFWwindow* window, double xoffset, double yoffset)
    {
        ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
        
        if (!ImGui::GetIO().WantCaptureMouse)
        {
            Zoom(yoffset * 50);
        }
    }
    static void glfw_mouse(GLFWwindow* window, int button, int action,
                           int mods)
    {
        ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
        if (!ImGui::GetIO().WantCaptureMouse)
        {
            mouse_buttons[button] = (action == GLFW_PRESS);
        }
    }                           
    bool Initialize(int width, int height, const std::string &window_name)
    {
        w_width = width;
        w_height = height;
        glfwSetErrorCallback(glfw_error_callback);
        if (!glfwInit())
            return 0;

        // Decide GL+GLSL versions
    #ifdef __APPLE__
        // GL 3.2 + GLSL 150
        const char* glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
    #else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
    #endif

        // Create window with graphics context
        window = glfwCreateWindow(w_width, w_height, window_name.c_str(), NULL, NULL);
        if (window == NULL)
            return 0;
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1); // Enable vsync

        // Initialize OpenGL loader
        GLenum err = glewInit();
        if (err != GLEW_OK)
        {
            std::cout<<RED<< "[ERROR]::[Visualizer]::Error initializing GLEW: " << glewGetErrorString(err)
                    << RESET<<std::endl;
            return false;
        }
        else
        {
            std::cout<<BLUE<<"[Visualizer]::[INFO]::GLEW init done."<<RESET<<std::endl;
        }

        glViewport( 0, 0, w_width, w_height);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);

        glEnable(GL_POINT_SMOOTH);
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
        glEnable (GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

        // Setup Dear ImGui style
        //ImGui::StyleColorsDark();
        ImGui::StyleColorsClassic();

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(glsl_version);
        // Load Fonts
        // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
        // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
        // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
        // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
        // - Read 'docs/FONTS.md' for more instructions and details.
        // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
        //io.Fonts->AddFontDefault();
        //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
        //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
        //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
        //io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
        //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
        //IM_ASSERT(font != NULL);
        return true;
    }
    void Cleanup()
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(window);
        glfwTerminate();
    }
    void RenderGuiComponents()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();        
        {
            ImGui::Begin("Menu");                         
            //ImGui::SameLine();
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImGui::Text("x: %f, y: %f", p.x, p.y);
            ImGui::End();
        }
        // Rendering
        ImGui::Render();
    }
    void Translate(const geometry::Vector3 &translation)
    {
        geometry::Vector3 tmp_t = model_view_matrix.block<3, 1>(0, 3) + translation;
        model_view_matrix.block<3, 1>(0, 3) = tmp_t;
    }
    void Rotate(const geometry::Matrix3 &rotation)
    {
        geometry::Matrix3 rot = rotation * model_view_matrix.block<3, 3>(0, 0);
        model_view_matrix.block<3, 3>(0, 0) = rot;
    }
    void Zoom(double y)
    {
        Translate(geometry::Vector3(0.0, 0.0, y * 3.0 / w_height));
    }
    void RegisterMouseAndKeyboard()
    {
        // glfwSetKeyCallback(window, glfw_keyboard);
        // std::cout<<"??? "<<std::endl;
        glfwSetCursorPosCallback(window, glfw_motion);
        glfwSetMouseButtonCallback(window, glfw_mouse);
        glfwSetScrollCallback(window, glfw_scroll);
    }
    void CursorPos(double& x, double& y)
    {
        glfwGetCursorPos(window, &x, &y);
    }
    void Translate(double x, double y)
    {
        double dx = x - last_x;
        double dy = y - last_y;
        geometry::Point3 center = bb.Center();
        geometry::Vector4 mc = geometry::Vector4(center(0), center(1), center(2), 1.0);
        geometry::Vector4 ec = model_view_matrix * mc;
        double z = -(ec[2] / ec[3]);

        Translate(geometry::Vector3(2.0 * dx / w_width * right / near * z,
                    -2.0 * dy / w_height * up / near * z, 0.0f));
    }
    bool Map2Sphere(const geometry::Point2i& point2D, geometry::Vector3& result)
    {
        if ((point2D[0] >= 0) && (point2D[0] <= w_width) && (point2D[1] >= 0) &&
            (point2D[1] <= w_height))
        {

            double x = (double)(point2D[0] - 0.5 * w_width) / w_width;
            double y = (double)(0.5 * w_height - point2D[1]) / w_height;
            double sinx = sin(M_PI * x * 0.5);
            double siny = sin(M_PI * y * 0.5);
            double sinx2siny2 = sinx * sinx + siny * siny;
            result[0] = sinx;
            result[1] = siny;
            result[2] = sinx2siny2 < 1.0 ? sqrt(1.0 - sinx2siny2) : 0.0;

            return true;
        }
        else
            return false;
    }
    void Rotate(const geometry::Vector3& axis, double angle)
    {
        // center in eye coordinates
        geometry::Point3 center = bb.Center();
        geometry::Vector4 mc = geometry::Vector4(center(0), center(1), center(2), 1.0);
        geometry::Vector4 ec = model_view_matrix * mc;
        geometry::Vector3 c(ec[0] / ec[3], ec[1] / ec[3], ec[2] / ec[3]);
        geometry::Matrix4 transform = geometry::Matrix4::Identity();
        geometry::Matrix3 rotation = geometry::RotationMatrix(axis, angle);
        geometry::Vector3 translation = c + rotation * (-c);
        transform.block<3, 3>(0, 0) = rotation;
        transform.block<3, 1>(0, 3) = translation;
        model_view_matrix = transform* model_view_matrix;
    }
    void Rotate(double x, double y, int type)
    {
        if(last_point_valid)
        {
            geometry::Point2i new_point_2d((int)x, (int)y);
            geometry::Point3 new_point_3d;
            bool new_point_valid = Map2Sphere(new_point_2d, new_point_3d);
            if (new_point_valid)
            {
                if(type == 0)
                {
                    auto axis = last_point_3d.cross(new_point_3d);
                    
                    double cos_ = geometry::Cos(last_point_3d, new_point_3d);

                    if (std::fabs(cos_) < 1.0)
                    {
                        double angle = 2.0 * acos(cos_);
                        Rotate(axis, angle);
                    }
                }
                else
                {
                    //z axis is the look-at direction, because it's model view matrix
                    // the look-at direction is just (0,0, 1)
                    auto axis = last_point_3d.cross(new_point_3d);
                    geometry::Vector3 z_axis = geometry::Vector3(0, 0, 1);
                    if(axis.dot(geometry::Vector3(0, 0, 1)) < 0) z_axis = -z_axis;
                    double cos_ = geometry::Cos(last_point_3d, new_point_3d);

                    if (std::fabs(cos_) < 1.0)
                    {
                        double angle = 2.0 * acos(cos_);
                        
                        Rotate(z_axis, angle);
                    }
                }
            }
        }
    }
    void Movement(double xpos, double ypos)
    {
        // rotation0
        if (right_mouse_pressed() && left_mouse_pressed())
        {
            Rotate(xpos, ypos, 1);
        }
        else if (right_mouse_pressed())
        {
            Rotate(xpos, ypos, 0);
        }
        // translation
        else if (left_mouse_pressed())
        {
            Translate(xpos, ypos);
        }
        // // zoom
        // else if (middle_mouse_pressed())
        // {
        //     Zoom(ypos);
        // }
        // remember points
        
        last_x = xpos;
        last_y = ypos;
        last_point_valid = Map2Sphere(geometry::Point2i((int)last_x, (int)last_y), last_point_3d);
    }
    void DrawCircle(const geometry::Point2 &p, double r, const Eigen::Vector3f &color, double thickness, int n)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINE_LOOP);              // Each set of 4 vertices form a quad
        for(int i = 0; i < n; ++i)
        {
            glVertex2f(p(0) +  cos((i + 0.0f) / n * 2 * M_PI) * r, (p(1) + sin((i + 0.0f) / n * 2 * M_PI) * r));
        }
        glEnd();
    }  
    void DrawCircleFilled(const geometry::Point2 &p, double r, const Eigen::Vector3f &color, int n)
    {
        glColor3f(color(0), color(1), color(2));
        glBegin(GL_POLYGON);              // Each set of 4 vertices form a quad
        for(int i = 0; i < n; ++i)
        {
            glVertex2f(p(0) +  cos((i + 0.0f) / n * 2 * M_PI) * r, (p(1) + sin((i + 0.0f) / n * 2 * M_PI) * r));
        }
        glEnd();
    }      
    void DrawPoint(const geometry::Point2 &p, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glPointSize(thickness);
        glBegin(GL_POINTS);              // Each set of 4 vertices form a quad
        glVertex2f(p(0), p(1));
        glEnd();
    }
    void DrawPoints(const geometry::Point2List &points, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glPointSize(thickness);
        glBegin(GL_POINTS);              // Each set of 4 vertices form a quad
        for(size_t i = 0; i < points.size(); ++i)
        glVertex2f(points[i](0), points[i](1));
        glEnd();
    }
    void DrawLine(const geometry::Point2 &p1, const geometry::Point2 &p2, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINES);
        glVertex2f(p1(0), p1(1));
        glVertex2f(p2(0), p2(1));
        glEnd();
    }
    void DrawLines(const geometry::Point2List &points, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINES);
        for(size_t i = 0; i < points.size(); i+=2)
        {
            glVertex2f(points[i](0), points[i](1));
            glVertex2f(points[i+1](0), points[i+1](1));
        }
        glEnd();
    }
    void DrawLineStrip(const geometry::Point2List &points, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINE_STRIP);
        for(size_t i = 0; i < points.size(); i+=2)
        {
            glVertex2f(points[i](0), points[i](1));
        }
        glEnd();
    }
    void DrawPolygon(const geometry::Point2List &points, const Eigen::Vector3f &color,  double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINE_LOOP);              
        for(size_t i = 0; i < points.size(); ++i)
        {
            // std::cout<< points[i].transpose()<<std::endl;
            glVertex2f(points[i](0), points[i](1));
        }
        glEnd();        
    }
    void DrawPolygonFilled(const geometry::Point2List &points, const Eigen::Vector3f &color)
    {
        glColor3f(color(0), color(1), color(2));
        glBegin(GL_POLYGON);              
        for(size_t i = 0; i < points.size(); ++i)
        {
            glVertex2f(points[i](0), points[i](1));
        }
        glEnd();        
    }


     
    void DrawPoint(const geometry::Point3 &p, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glPointSize(thickness);
        glBegin(GL_POINTS);              // Each set of 4 vertices form a quad
        glVertex3f(p(0), p(1), p(2));
        glEnd();
    }
    void DrawPoints(const geometry::Point3List &points, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glPointSize(thickness);
        glBegin(GL_POINTS);              // Each set of 4 vertices form a quad
        for(size_t i = 0; i < points.size(); ++i)
        glVertex3f(points[i](0), points[i](1), points[i](2));
        glEnd();
    }
    void DrawLine(const geometry::Point3 &p1, const geometry::Point3 &p2, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINES);
        glVertex3f(p1(0), p1(1), p1(2));
        glVertex3f(p2(0), p2(1), p2(2));
        glEnd();
    }
    void SetProjectionMatrix()
    {
        glMatrixMode(GL_PROJECTION);
#ifdef USING_FLOAT64
        glLoadMatrixd(projection_matrix.data());
#else
        glLoadMatrixf(projection_matrix.data());
#endif
    }
    void SetModelViewMatrix()
    {
        glMatrixMode(GL_MODELVIEW);
#ifdef USING_FLOAT64
        glLoadMatrixd(model_view_matrix.data());
#else
        glLoadMatrixf(model_view_matrix.data());
#endif
    }
    void DrawLines(const geometry::Point3List &points, const Eigen::Vector3f &color, double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINES);
        for(size_t i = 0; i < points.size(); i+=2)
        {
            glVertex3f(points[i](0), points[i](1), points[i](2));
            glVertex3f(points[i+1](0), points[i+1](1), points[i+1](2));
        }
        glEnd();
    }
    void DrawLineStrip(const geometry::Point3List &points, const Eigen::Vector3f &color, double thickness)
    {
        
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINE_STRIP);
        for(size_t i = 0; i < points.size(); i+=2)
        {
            glVertex3f(points[i](0), points[i](1), points[i](2));
        }
        glEnd();
    }
    void DrawPolygon(const geometry::Point3List &points, const Eigen::Vector3f &color,  double thickness)
    {
        glColor3f(color(0), color(1), color(2));
        glLineWidth(thickness);
        glBegin(GL_LINE_LOOP);              
        for(size_t i = 0; i < points.size(); ++i)
        {
            // std::cout<< points[i].transpose()<<std::endl;
            glVertex3f(points[i](0), points[i](1), points[i](2));
        }
        glEnd();        
    }
    void DrawPolygonFilled(const geometry::Point3List &points, const Eigen::Vector3f &color)
    {
        glColor3f(color(0), color(1), color(2));
        glBegin(GL_POLYGON);              
        for(size_t i = 0; i < points.size(); ++i)
        {
            glVertex3f(points[i](0), points[i](1), points[i](2));
        }
        glEnd();        
    }
}
}
}
