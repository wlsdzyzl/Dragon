#include "Visualization/Visualizer.h"
#include "Geometry/TriangleMesh/Processing/Smoothing.h"
#include "Geometry/TriangleMesh/Processing/Curvature.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
#include "Geometry/TriangleMesh/Processing/Subdivision.h"
#include "Tool/ColorMapping.h"
#include "Reconstruction/RBF.h"
#include "Reconstruction/Poisson.h"
#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/MeshParameterization.h"
using namespace dragon;
static float local_lambda = 0.1f;
static float global_lambda = 0.1f;
static int iteration_local = 200;
static bool use_uniform_para = true;
static geometry::TriangleMesh object;
static geometry::PointCloud pcd;
static geometry::Octree oct;
static bool is_mesh = true;
static std::string filename;
static bool wireframe_mode = false;
static float simplify_rate = 0.1f;
static float voxel_len = 0.01f;
static char voxel_len_s[100] = "0.01";
static double radius = 0.5;
static char radius_s[100] = "0.5";
static int knn = 10;
static char knn_s[100] = "10";
static double scale = 1.0;
static char scale_s[100] = "1.0";
static bool r_normal = false;
bool updated = false;
bool reorient = false;
bool draw_octree = false;
static int max_depth = 5;
visualization::Visualizer visualizer(1000, 750);
void RenderGuiComponents()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();        
    {
        ImGui::Begin("Menu");                         
        //ImGui::SameLine();
        if(is_mesh)
        {
            ImGui::Checkbox("Wireframe", &wireframe_mode); 
            ImGui::SameLine();
            ImGui::Checkbox("NormalMapping", &visualizer.draw_normal);
            ImGui::Checkbox("ColorMapping", &visualizer.draw_color);
            ImGui::SameLine();
            ImGui::Checkbox("PhongShading", &visualizer.draw_phong_shading); 
            // ImGui::BeginMenu("Curvature");
            if (ImGui::CollapsingHeader("Curvature", ImGuiTreeNodeFlags_None))
            {
                if(ImGui::Button("Mean Curvature"))
                {
                    geometry::HalfEdge he; 
                    he.FromTriangleMesh(object);
                    he.CheckBorder();
                    geometry::ScalarList mean_curvatures;
                    geometry::mesh::ComputeMeanCurvature(he, mean_curvatures);
                    // for(int i = 0; i != mean_curvatures.size(); ++i)
                    // std::cout<<mean_curvatures[i]<<std::endl;
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
                    geometry::ScalarList gauss_curvatures;
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
            if(ImGui::CollapsingHeader("Smoothing", ImGuiTreeNodeFlags_None))
            {
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
            }
             
            if(ImGui::CollapsingHeader("Parameterization", ImGuiTreeNodeFlags_None))
            {
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
            }
            if(ImGui::CollapsingHeader("Simplification", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::SliderFloat("Percentage", &simplify_rate, 0.01f, 1.0f);
                if(ImGui::Button("Quadric Simplification"))
                {
                    // auto result = object.QuadricSimplify(object.triangles.size() - 1);
                    auto result = geometry::mesh::QuadricDecimation(object,  simplify_rate * object.triangles.size());
                    object = *result;
                    if(!object.HasNormals())
                    object.ComputeNormals();
                    updated = true;
                }
                if(ImGui::InputText("Grid Unit", voxel_len_s, 100))
                {
                    // std::cout << "grid unit: "<<voxel_len_s<<std::endl;
                    voxel_len = std::atof(voxel_len_s);
                }
                if(ImGui::Button("Clustering Simplification"))
                {
                    // auto result = object.ClusteringSimplify(voxel_len);
                    auto result = geometry::mesh::ClusteringDecimation(object,  voxel_len);
                    object = *result;
                    if(!object.HasNormals())
                    object.ComputeNormals();
                    updated = true;
                }   
            }
            if(ImGui::Button("Loop Subdivision"))
            {
                // auto result = object.QuadricSimplify(object.triangles.size() - 1);
                auto result = geometry::mesh::LoopSubdivision(object, 1);
                object = *result;
                if(!object.HasNormals())
                object.ComputeNormals();
                updated = true;
            }
            if(ImGui::Button("Generate SDF"))
            {
                is_mesh = false;
                scale = 1 / (std::max(visualization::window::bb.y_max - visualization::window::bb.y_min, 
                    std::max(visualization::window::bb.x_max - visualization::window::bb.x_min, 
                        visualization::window::bb.z_max - visualization::window::bb.z_min)));
                object.Scale(scale);
                reconstruction::CubeHandler cube_handler;
                reconstruction::Mesh2SDF(object, cube_handler, 0.01);
                pcd = *(cube_handler.GetPointCloud());
                pcd.Scale(1 / scale);
                pcd.normals.resize(pcd.points.size(), geometry::Point3(0, 0, 0));

                object.Reset();
                updated = true;
            }  
            if(ImGui::Button("Get PointCloud"))
            {
                is_mesh = false;
                pcd = *(object.GetPointCloud());
                // for(size_t i = 0; i != pcd.colors.size(); ++i)
                // std::cout<<pcd.colors[i].transpose()<<std::endl;
                object.Reset();
                updated = true;
            }     
 
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::Text("triangle: %d vertices: %d", (int)object.triangles.size(), (int)object.points.size());
        }
        else
        {
            ImGui::Checkbox("NormalMapping", &visualizer.draw_normal);
            ImGui::SameLine();
            ImGui::Checkbox("ColorMapping", &visualizer.draw_color);
            ImGui::SameLine();
            ImGui::Checkbox("PhongShading", &visualizer.draw_phong_shading); 
            if(ImGui::CollapsingHeader("Normal Operation", ImGuiTreeNodeFlags_None))
            {
                if(ImGui::InputText("Radius", radius_s, 100))
                {
                    // std::cout << "grid unit: "<<voxel_len_s<<std::endl;
                    radius = std::atof(radius_s);
                }
                if(ImGui::InputText("KNN", knn_s, 100))
                {
                    // std::cout << "grid unit: "<<voxel_len_s<<std::endl;
                    knn = std::atoi(knn_s);
                }
                if(ImGui::Button("Estimate Normals"))
                {
                    pcd.EstimateNormals(radius, knn);
                    updated = true;
                }
                if(ImGui::Button("Flip Normals"))
                {
                    pcd.FlipNormal();
                    updated = true;
                }  
            }
            if(ImGui::CollapsingHeader("Processing", ImGuiTreeNodeFlags_None))
            {
                if(ImGui::InputText("Grid Unit", voxel_len_s, 100))
                {
                    // std::cout << "grid unit: "<<voxel_len_s<<std::endl;
                    voxel_len = std::atof(voxel_len_s);
                }
                if(ImGui::Button("Down Sample"))
                {

                    pcd = *(pcd.DownSample(voxel_len));
                    updated = true;
                }
                if(ImGui::InputText("Scale", scale_s, 100))
                {
                    // std::cout << "grid unit: "<<voxel_len_s<<std::endl;
                    scale = std::atof(scale_s);
                }
                if(ImGui::Button("Scaling"))
                {
                    pcd.Scale(scale);
                    updated = true;
                }
            }
            if(ImGui::CollapsingHeader("Reconstruction", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Recompute Normal", &r_normal);
                // ImGui::SameLine();
                if(ImGui::Button("RBF Reconstruction"))
                {
                    scale = 2 / (std::max(visualization::window::bb.y_max - visualization::window::bb.y_min, 
                        std::max(visualization::window::bb.x_max - visualization::window::bb.x_min, 
                            visualization::window::bb.z_max - visualization::window::bb.z_min)));
                    // std::cout<<scale<<std::endl;
                    pcd.Scale(scale);
                    if(r_normal)
                    pcd.EstimateNormals(1, 10); 
                    auto d_pcd = pcd.DownSample(0.16);
                    pcd = *d_pcd;
                    reconstruction::CubeHandler cube_handler;
                    cube_handler.SetTruncation(0.4);
                    cube_handler.SetVoxelResolution(0.05);

                    auto result = reconstruction::RBF(pcd, cube_handler, 0.1);
                    object = *(geometry::mesh::ClusteringDecimation(*result, 0.05));
                    object.ComputeNormals();
                    is_mesh = true;
                    pcd.Reset();
                    updated = true;
                    reorient = true;
                    draw_octree = false;
                }    
                ImGui::SliderInt("Octree Depth", &max_depth, 1, 8);
                if(ImGui::Button("Draw Octree"))
                {
                    oct.Reset();
                    oct.max_depth = max_depth;
                    oct.BuildTree(pcd);
                    updated = true;
                    draw_octree = true;
                } 
                if(ImGui::Button("Poisson Reconstruction"))
                {
                    scale = 2 / (std::max(visualization::window::bb.y_max - visualization::window::bb.y_min, 
                        std::max(visualization::window::bb.x_max - visualization::window::bb.x_min, 
                            visualization::window::bb.z_max - visualization::window::bb.z_min)));
                    // std::cout<<scale<<std::endl;
                    pcd.Scale(scale);
                    if(r_normal)
                    pcd.EstimateNormals(1, 10); 
                    auto result = reconstruction::Poisson(pcd,max_depth);
                    object = *(geometry::mesh::ClusteringDecimation(*result, 0.05));
                    object.FlipNormal();
                    object.ComputeNormals();
                    is_mesh = true;
                    pcd.Reset();
                    updated = true;
                    reorient = true;
                    draw_octree = false;
                }       
            }
            ImGui::Text("points: %d", (int)pcd.points.size());
        
        }
        if(ImGui::Button("Reload"))
        {
            is_mesh = true;
            draw_octree = false;
            object.LoadFromFile(filename);
            if(!object.triangles.size())
            {
                is_mesh = false;
                pcd = *(object.GetPointCloud());
                object.Reset();
            }

            if(is_mesh)
            {
                if(!object.HasColors())
                object.colors = geometry::Point3List(object.points.size(), geometry::Point3(0.7, 0.7, 0.7));
                if(!object.HasNormals())
                object.ComputeNormals();
            }
            else
            {
                if(!pcd.HasColors())
                pcd.colors = geometry::Point3List(pcd.points.size(), geometry::Point3(0.7, 0.7, 0.7));
                if(!pcd.HasNormals())
                pcd.normals = geometry::Point3List(pcd.points.size(), geometry::Point3::Zero());   
            }
            updated = true;
            reorient = true;
        }
        ImGui::SameLine();
        if(ImGui::Button("Save"))
        {
            if(is_mesh)
            object.WriteToPLY("ByDragon.ply");
            else
            pcd.WriteToPLY("ByDragon.ply");
            std::cout << "save to ByDragon.ply."<<std::endl;
        }  
        ImGui::ColorEdit3("Background color", visualizer.clear_color.data());
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
    if(!object.triangles.size())
    {
        is_mesh = false;
        pcd = *(object.GetPointCloud());
    }
    if(is_mesh)
    {
        if(!object.HasColors())
        object.colors = geometry::Point3List(object.points.size(), geometry::Point3(0.7, 0.7, 0.7));

        if(!object.HasNormals())
        object.ComputeNormals();
    }
    else
    {
        if(!pcd.HasColors())
        pcd.colors = geometry::Point3List(pcd.points.size(), geometry::Point3(0.7, 0.7, 0.7));
        if(!pcd.HasNormals())
        pcd.normals = geometry::Point3List(pcd.points.size(), geometry::Point3::Zero());   
    }
    //visualizer.SetDrawColor(true);
    visualizer.Reset();
    visualizer.Initialize();
    if(is_mesh)
    visualizer.AddTriangleMesh(object);
    else
    visualizer.AddPointCloud(pcd);
    visualizer.draw_color = true;
    visualization::window::RegisterMouseAndKeyboard();
    visualizer.dynamic_first_view = false;
    // if(!is_mesh)    glDisable(GL_DEPTH_TEST);
    while(!glfwWindowShouldClose(visualization::window::window))
    {
        RenderGuiComponents();
        visualizer.wireframe_mode = wireframe_mode;
        if(updated)
        {
            visualizer.Reset();
            if(is_mesh)
            {
                // glEnable(GL_DEPTH_TEST);
                visualizer.AddTriangleMesh(object);
            }
            else
            {
                // glDisable(GL_DEPTH_TEST);
                visualizer.AddPointCloud(pcd);
                if(draw_octree)
                visualizer.AddOctree(oct);
            }
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