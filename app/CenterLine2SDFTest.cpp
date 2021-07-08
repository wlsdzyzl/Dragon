#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
using namespace dragon;
void ReadCenterLines(const std::string &path, geometry::Point3List &centers, std::vector<double> &radius)
{
    std::ifstream ifs(path);
    centers.clear();
    radius.clear();
    while(ifs)
    {
        geometry::Point3 center;
        double useless;
        double r;
        ifs >> center[0] >> center[1] >> center[2] >> r >> useless;
        // std::cout<<center.transpose()<<std::endl;
        centers.push_back(center);
        radius.push_back(r);
    }
    centers.pop_back();
    radius.pop_back();
    std::cout<<"number of centers: "<<centers.size()<<" "<<radius.size()<<std::endl;
}
int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout << "Usage: CenterLine2SDFTest [filename] [voxel resolution = 0.01]"<<std::endl;
        return 0;        
    }
    geometry::TriangleMesh generated_mesh;
    geometry::Point3List centers;
    std::vector<double> radius;
    ReadCenterLines(argv[1], centers, radius);
    
    // scaling
    geometry::BoundingBox bb;
    for(size_t i = 0; i != centers.size(); ++i)
    bb.AddPoint(centers[i]);
    double scale = 1 / (std::max(bb.y_max - bb.y_min, std::max(bb.x_max - bb.x_min, bb.z_max - bb.z_min)));
    for(size_t i = 0; i != centers.size(); ++i)
    {
        centers[i] *= scale;
        radius[i] *= scale;
    }
    float voxel_resolution = 0.01;
    if(argc > 2)
    voxel_resolution = std::atof(argv[2]);

    reconstruction::CubeHandler cube_handler;
    reconstruction::CenterLine2SDF(centers, radius, cube_handler, voxel_resolution);
    auto cube_pcd = cube_handler.GetPointCloud();
    cube_pcd->Scale(1 / scale);
    cube_pcd->WriteToPLY("./tsdf.ply");
    cube_handler.ExtractTriangleMesh(generated_mesh);
    generated_mesh.Scale(1 / scale);
    generated_mesh.WriteToPLY("./generated_mesh.ply");
    return 0;
}