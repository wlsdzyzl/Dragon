#include "Reconstruction/RBF.h"
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
    if(!pcd.HasNormals()) pcd.EstimateNormals(10, 10);
    pcd.WriteToPLY("./pcd.ply");
    std::cout<<BLUE<<"[INFO]::[RBF]::Estimate normal."<<RESET<<std::endl;
    geometry::BoundingBox bb;
    for(size_t i = 0; i != pcd.points.size(); ++i)
    bb.AddPoint(pcd.points[i]);
    std::cout<<bb.x_max<<" "<<bb.x_min<<std::endl;
    reconstruction::CubeHandler cube_handler;
    cube_handler.SetTruncation(0.05);
    cube_handler.SetVoxelResolution(0.02);
    // cube_handler.ReadFromFile("/media/wlsdzyzl/wlsdzyzl_2/OnePiece/build/example/tsdf.map");
    // geometry::TriangleMesh mesh;
    // cube_handler.ExtractTriangleMesh(mesh);
    // auto result = geometry::mesh::ClusteringDecimation(mesh, 0.01);
    // result->WriteToPLY("./RBF.ply");
    auto result = reconstruction::RBF(pcd, cube_handler);
    result = geometry::mesh::ClusteringDecimation(*result, 0.01);
    result->ComputeNormals();
    result->WriteToPLY("./RBF.ply");
    auto cube_pcd = cube_handler.GetPointCloud();
    cube_pcd->WriteToPLY("./tsdf.ply");
    return 0;
}