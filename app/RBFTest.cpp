#include "Reconstruction/RBF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
using namespace dragon;

int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cout << "Usage: RBFTest [filename]"<<std::endl;
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
    auto d_pcd = pcd.DownSample(0.16);
    pcd = *d_pcd;

    pcd.WriteToPLY("./pcd.ply");
    std::cout<<BLUE<<"[INFO]::[RBF]::Estimate normal."<<RESET<<std::endl;


    
    reconstruction::CubeHandler cube_handler;
    cube_handler.SetTruncation(0.32);
    cube_handler.SetVoxelResolution(0.04);
    // cube_handler.ReadFromFile("/media/wlsdzyzl/wlsdzyzl_2/OnePiece/build/example/tsdf.map");
    // geometry::TriangleMesh mesh;
    // cube_handler.ExtractTriangleMesh(mesh);
    // auto result = geometry::mesh::ClusteringDecimation(mesh, 0.01);
    // result->WriteToPLY("./RBF.ply");
    auto result = reconstruction::RBF(pcd, cube_handler, 0.1);
    result = geometry::mesh::ClusteringDecimation(*result, 0.02);
    result->ComputeNormals();
    result->colors.clear();
    result->WriteToPLY("./RBF.ply");
    auto cube_pcd = cube_handler.GetPointCloud();
    cube_pcd->WriteToPLY("./tsdf.ply");
    return 0;
}