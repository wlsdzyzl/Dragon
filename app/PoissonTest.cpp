#include "Reconstruction/Poisson.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc < 3)
    {
        std::cout << "Usage: PoissonTest [filename] [depth] [outputfile='poisson_$depth$.ply']"<<std::endl;
        return 0;        
    }
    geometry::PointCloud pcd;
    pcd.LoadFromFile(argv[1]);
    
    geometry::BoundingBox bb;
    for(size_t i = 0; i != pcd.points.size(); ++i)
    bb.AddPoint(pcd.points[i]);
       

    double scale = 2 / (std::max(bb.y_max - bb.y_min, std::max(bb.x_max - bb.x_min, bb.z_max - bb.z_min)));
    // std::cout<<bb.x_max<<" "<<bb.x_min<<std::endl;

    pcd.Scale(scale);
    if(!pcd.HasNormals()) pcd.EstimateNormals(1, 10); 
    auto d_pcd = pcd.DownSample(0.1);
    pcd = *d_pcd;
    int depth = atoi(argv[2]);
    
    std::string output = "./poisson_"+std::to_string(depth)+".ply";
    if(argc > 3) output = argv[3];
    // double offset = 1 / std::pow(2, depth);
    // pcd.Translate( - geometry::Point3(bb.x_max + bb.x_min, bb.y_max + bb.y_min, bb.z_max + bb.z_min)  * scale/ 2.0 + 
    //     geometry::Point3(offset, offset, offset));

    // pcd.FlipNormal();
    // pcd = *(pcd.DownSample(0.16));
    std::cout<<BLUE<<"[INFO]::[RBF]::Estimate normal."<<RESET<<std::endl;
    auto mesh_ptr = reconstruction::Poisson(pcd, depth);
    mesh_ptr->FlipNormal();
    mesh_ptr->ComputeNormals();
    mesh_ptr->WriteToPLY(output);

    // auto n_mesh_ptr = reconstruction::NaivePoisson(pcd, depth);
    // n_mesh_ptr->FlipNormal();
    // n_mesh_ptr->ComputeNormals();
    // n_mesh_ptr->WriteToPLY("./naive_poisson_"+std::to_string(depth)+".ply");
    return 0;
}