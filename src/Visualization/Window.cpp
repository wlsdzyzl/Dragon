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
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);   
    int w_width;
    int w_height;
    double last_x = 0;
    double last_y = 0;
    static void glfw_error_callback(int error, const char* description)
    {
        std::cout<<RED<<"[ERROR]::[Window]::Glfw Error: "<<description<<RESET<<std::endl;
    }  

    //callback function when we move mouse and keyboard
    static void glfw_motion(GLFWwindow* window, double xpos, double ypos)
    {
        movement(xpos, ypos);
    }
    static void glfw_scroll(GLFWwindow* window, double xoffset, double yoffset)
    {
        ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
        if (!ImGui::GetIO().WantCaptureMouse)
        {
            Zoom(yoffset);
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
                              
    bool Initialize(int width, int height)
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
        window = glfwCreateWindow(w_width, w_height, "Dragon 3D", NULL, NULL);
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
            std::cout<<BLUE<<"[Visualizer]::[INFO]::GLEW init done."<<std::endl;
        }

        glViewport( 0, 0, w_width, w_height);
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
            ImGui::Begin("Menu");                          // Create a window called "Hello, world!" and append into it.
            ImGui::Checkbox("Phong Lighting", &show_demo_window);      // Edit bools storing our window open/close state

            if(ImGui::Button("Minimal Surface"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
            {

            }
            if(ImGui::Button("Mean Curvature"))
            {

            }
            if(ImGui::Button("Gauss Curvature"))
            {

            }
            double x, y;
            CursorPos(x, y);
            {
                ImGui::Text("Cursor: %f/%f", x, y);
            }
            //ImGui::SameLine();
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
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
        float dy = y - last_y;
        Translate(geometry::Vector3(0.0, 0.0, dy * 3.0 / w_height));
    }
    void RegisterMouseAndKeyboard()
    {
        // glfwSetKeyCallback(window, glfw_keyboard);
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
        // float dx = x - last_x;
        // float dy = y - last_y;

        // vec4 mc = vec4(center, 1.0);
        // vec4 ec = modelview_matrix * mc;
        // float z = -(ec[2] / ec[3]);

        // float aspect = (float)w_width / (float)w_height;
        // float up = tan(fovy_ / 2.0f * M_PI / 180.f) * near;
        // float right = aspect * up;

        // Translate(geometry::Vector3(2.0 * dx / w_width * right / near * z,
        //             -2.0 * dy / w_height * up / near * z, 0.0f));
    }
    void Rotate(double x, double y, int type)
    {
        
    }
    void movement(double xpos, double ypos)
    {
        // rotation0
        if (right_mouse_pressed() && left_mouse_pressed())
        {
            Rotate(xpos, ypos, 2);
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
        // zoom
        else if (middle_mouse_pressed())
        {
            Zoom(ypos);
        }
        // remember points
        last_x = xpos;
        last_y = ypos;
    }
}
}
}
