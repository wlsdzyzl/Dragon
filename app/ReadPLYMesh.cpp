#include "Visualization/Visualizer.h"
using namespace dragon;
int main(int argc, char* argv[]) 
{
    if(argc != 2)
    {
        std::cout << "Usage: ReadPLYMesh [filename.ply]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromFile(argv[1]);

    // if(!mesh.HasNormals())
    //     mesh.ComputeNormals();
    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();
    Eigen::Matrix3d R = geometry::RandomTransformation().block<3, 3>(0, 0).cast<double>();
    Eigen::Matrix3d axis;
    axis << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    Eigen::Matrix3d transformed_R =  R * axis ;
    std::cout<< transformed_R.transpose() - transformed_R.inverse()<<std::endl;
    Eigen::Quaterniond q(transformed_R); 
    geometry::PointCloud pcd_a = *mesh.GetPointCloud();
    geometry::PointCloud pcd_b = pcd_a;
    for(size_t i = 0; i != pcd_a.points.size(); ++i)
    {
        pcd_a.points[i] = transformed_R *  pcd_a.points[i];
    }
    for(size_t i = 0; i != pcd_b.points.size(); ++i)
    {
        pcd_b.points[i] = q *  pcd_b.points[i];
    }

    pcd_a.WriteToPLY("./rotation_matrix.ply");
    pcd_b.WriteToPLY("./quaterniond_pcd.ply");
    // mesh.WriteToPLY("./dragon.ply");
    return 0;
}