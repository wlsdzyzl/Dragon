#include "Reconstruction/Poisson.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cout << "Usage: ReadPLYMesh [filename]"<<std::endl;
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
    // pcd.FlipNormal();
    // pcd = *(pcd.DownSample(0.16));
    std::cout<<BLUE<<"[INFO]::[RBF]::Estimate normal."<<RESET<<std::endl;
    reconstruction::Poisson(pcd, 6);
    return 0;
}