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
        // double useless;
        double r;
        ifs >> center[0] >> center[1] >> center[2] >> r;
        // std::cout<<center.transpose()<<std::endl;
        centers.push_back(center);
        // r = r<1.0?1.0:r;
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
        std::cout << "Usage: CenterLine2SDFTest [input_filename] [output_filename] [voxel resolution = 0.01] [ordered = 1] [radius_factor = -1.0] [scaling = -1]"<<std::endl;
        return 0;        
    }

    geometry::TriangleMesh generated_mesh;
    geometry::Point3List centers;
    std::vector<double> radius;
    ReadCenterLines(argv[1], centers, radius);
    std::string output_filename = argv[2];
    double scale = -1;
    double radius_factor = -1.0;
    float voxel_resolution = 0.01;
    int ordered = 1;
    if(argc > 3)
    voxel_resolution = std::atof(argv[3]);
    if(argc > 4)
    ordered = std::atoi(argv[4]);
    if(argc > 5)
    radius_factor = std::atof(argv[5]);
    if(argc > 6)
    scale = std::atof(argv[6]);
    // scaling
    // why is scaling so important?
    if(scale <= 0)
    {
        geometry::BoundingBox bb;
        for(size_t i = 0; i != centers.size(); ++i)
        bb.AddPoint(centers[i]);
        scale = 1 / (std::max(bb.y_max - bb.y_min, std::max(bb.x_max - bb.x_min, bb.z_max - bb.z_min)));
        // std::cout<<scale<<" "<<bb.y_max <<" "<< bb.y_min<<" " <<bb.x_max <<" "<< bb.x_min <<" "<<bb.z_max <<" "<< bb.z_min <<std::endl;
        
    }
    for(size_t i = 0; i != centers.size(); ++i)
    {
        centers[i] *= scale;
        radius[i] *= scale;
    }

    reconstruction::CubeHandler cube_handler;
    cube_handler.SetTruncation(voxel_resolution * 5);

    // // pre-processing: voxel clustering
    // {
    //     auto clusters = geometry::VoxelClustering(centers, voxel_resolution * 2);
    //     geometry::Point3List ccenters;
    //     std::vector<double> cradius;
    //     for(auto &c: clusters)
    //     {
    //         geometry::Point3 tmp_center = geometry::Point3::Zero();
    //         double tmp_r = 0;
    //         for(auto &id: c)
    //         {
    //             tmp_center += centers[id];
    //             tmp_r += radius[id];
    //         }
    //         ccenters.push_back(tmp_center / c.size());
    //         cradius.push_back(tmp_r / c.size());
    //     }
    //     centers = ccenters;
    //     radius = cradius;
    // }
    // pre-processing: radius clustering
    if(radius_factor > 100)
    {
        geometry::Point3List ccenters;
        std::vector<double> cradius;
        auto clusters = geometry::RadiusClustering(centers, radius, radius_factor);
        for(auto &c: clusters)
        {
            geometry::Point3 tmp_center = geometry::Point3::Zero();
            double tmp_r = 0;
            for(auto &id: c)
            {
                tmp_center += centers[id];
                tmp_r += radius[id];
            }
            ccenters.push_back(tmp_center / c.size());
            cradius.push_back(tmp_r / c.size());
        }
        centers = ccenters;
        radius = cradius;    
    }    


    cube_handler.SetVoxelResolution(voxel_resolution);
    reconstruction::CenterLine2SDF(centers, radius, cube_handler, voxel_resolution, ordered, 2);


    //for(size_t i = 0; i != centers.size(); ++i)
    //{
      //  centers[i] /= scale;
    //}
    //geometry::PointCloud pcd;
    //pcd.points = centers;
    //pcd.WriteToPLY(output_filename+".centers.ply");

    cube_handler.ExtractTriangleMesh(generated_mesh);
    auto simplified_mesh = geometry::mesh::ClusteringDecimation(generated_mesh, voxel_resolution);
    auto res_mesh = geometry::mesh::ToManifoldMesh(*simplified_mesh);
    res_mesh->Scale(1.0 / scale);
    res_mesh-> WriteToPLY(output_filename);

    return 0;
}
