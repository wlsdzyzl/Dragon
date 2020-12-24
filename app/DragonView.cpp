#include "Visualization/Visualizer.h"
#include "Geometry/TriangleMesh/Processing/Smoothing.h"
#include "Geometry/TriangleMesh/Processing/Curvature.h"
#include "Tool/ColorMapping.h"
#include "Geometry/TriangleMesh/Processing/MeshParameterization.h"
using namespace dragon;
static float local_lambda = 0.1f;
static float global_lambda = 0.1f;
static int iteration_local = 200;
static bool use_uniform_para = true;
static geometry::mesh::TriangleMesh object;
static std::string filename;
static bool wireframe_mode = false;
static double simplify_rate = 0.1f;
static double voxel_len = 0.01f;
bool updated = false;
bool reorient = false;
bool show_normal = false;
void RenderGuiComponents()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();        
    {
        ImGui::Begin("Menu");                         
        //ImGui::SameLine();
        ImGui::Checkbox("Wireframe", &wireframe_mode); 
        ImGui::SameLine();
        ImGui::Checkbox("NormalMapping", &show_normal);
        // ImGui::BeginMenu("Curvature");
        if(ImGui::Button("Mean Curvature"))
        {
            geometry::HalfEdge he; 
            he.FromTriangleMesh(object);
            he.CheckBorder();
            std::vector<double> mean_curvatures;
            geometry::mesh::ComputeMeanCurvature(he, mean_curvatures);
            geometry::Point3List colors;
            tool::ColorRemapping(mean_curvatures, colors);
            for(size_t i = 0; i != he.vertices.size(); ++i)
            {   
                he.vertices[i]->color = colors[i];
            }    
    
            he.has_colors = true;
            he.ToTriangleMesh(object);

            if(!object.HasNormals())
            object.ComputeNormals();
            updated = true;
        }
        if(ImGui::Button("Gauss Curvature"))
        {
            geometry::HalfEdge he; 
            he.FromTriangleMesh(object);
            he.CheckBorder();
            std::vector<double> gauss_curvatures;
            geometry::mesh::ComputeGaussCurvature(he, gauss_curvatures);
            geometry::Point3List colors;
            tool::ColorRemapping(gauss_curvatures, colors);
            for(size_t i = 0; i != he.vertices.size(); ++i)
            {   
                he.vertices[i]->color = colors[i];
            }    
            he.has_colors = true;
            he.ToTriangleMesh(object);
            if(!object.HasNormals())
            object.ComputeNormals();
            updated = true;
        }
        // if(ImGui::Button("Naive Laplacian Smoothing"))
        // {
        //     auto result = 
        //         geometry::mesh::NaiveLaplacianSmooting(object, local_lambda, iteration_local);            
        //     object = *result;
        //     if(!object.HasNormals())
        //     object.ComputeNormals();
        //     updated = true;
        // }
        ImGui::SliderFloat("local lambda", &local_lambda, 0.001f, 0.5f);
        ImGui::SliderInt("Local Iteration", &iteration_local, 1, 1000);
        if(ImGui::Button("Local Laplacian Smoothing"))
        {
            auto result = 
                geometry::mesh::LocalLaplacianSmooting(object, local_lambda, iteration_local);            
            object = *result;
            if(!object.HasNormals())
            object.ComputeNormals();
            updated = true;
        }
        ImGui::SliderFloat("global lambda", &global_lambda, 0.01f, 1.0f);
        if(ImGui::Button("Global Laplacian Smoothing"))
        {
            auto result = 
                geometry::mesh::GlobalLaplacianSmooting(object, 1 - global_lambda);            
            object = *result;
            if(!object.HasNormals())
            object.ComputeNormals();
            updated = true;
        }
        ImGui::Checkbox("Uniform", &use_uniform_para); 
        if(ImGui::Button("UV Parameterization (Circle)"))
        {
            if(use_uniform_para)
            {
                auto result = 
                    geometry::mesh::MeshParameterizationCircle(object, 0);            
                object = *result;
            }
            else
            {
                auto result = 
                    geometry::mesh::MeshParameterizationCircle(object, 3);            
                object = *result;
            }
            if(!object.HasNormals())
            object.ComputeNormals();
            reorient = true;
            updated = true;
        }
        if(ImGui::Button("UV Parameterization (Square)"))
        {
            if(use_uniform_para)
            {
                auto result = 
                    geometry::mesh::MeshParameterizationSquare(object, 0);            
                object = *result;
            }
            else
            {
                auto result = 
                    geometry::mesh::MeshParameterizationSquare(object, 3);            
                object = *result;
            }
            if(!object.HasNormals())
            object.ComputeNormals();
            reorient = true;
            updated = true;
        }
        if(ImGui::Button("Quadric Simplification"))
        {
            auto result = object.QuadricSimplify(simplify_rate * object.triangles.size());
            object = *result;
            if(!object.HasNormals())
            object.ComputeNormals();
            updated = true;
        }
        if(ImGui::Button("Clustering Simplification"))
        {
            auto result = object.QuadricSimplify(voxel_len);
            object = *result;
            if(!object.HasNormals())
            object.ComputeNormals();
            updated = true;
        }
        if(ImGui::Button("Reload"))
        {
            object.LoadFromFile(filename);
            if(!object.HasNormals())
            object.ComputeNormals();
            updated = true;
            reorient = true;
        }
        ImGui::SameLine();
        if(ImGui::Button("Save"))
        {
            object.WriteToPLY("ByDragon.ply");
            std::cout << "save to ByDragon.ply."<<std::endl;
        }        
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }
    // Rendering
    ImGui::Render();
}
int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cout << "Usage: DragonView [filename]"<<std::endl;
        return 0;
    }
    filename = argv[1];
    object.LoadFromFile(filename);
    if(!object.HasNormals())
    object.ComputeNormals();
    visualization::Visualizer visualizer(1000, 750);
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(object);
    visualizer.Initialize();
    visualization::window::RegisterMouseAndKeyboard();
    visualizer.dynamic_first_view = false;
    while(!glfwWindowShouldClose(visualization::window::window))
    {
        RenderGuiComponents();
        visualizer.wireframe_mode = wireframe_mode;
        if(show_normal)
        {
            visualizer.draw_normal = true;
            visualizer.draw_color_phong = false;
        }
        else
        visualizer.draw_color_phong = true;
        if(updated)
        {
            visualizer.Reset();
            visualizer.AddTriangleMesh(object);
            updated = false;
            if(reorient)
            {
                visualizer.ChooseCameraPoseThroughBB(visualization::window::bb);
                visualizer.SetModelViewMatrix(visualizer.camera_pose_for_view);
                reorient = false;
            }
        }
        visualizer.ShowOnce();
    }
    return 0;    
}