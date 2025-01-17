#include "Reconstruction/Mesh2SDF.h"
#include "Geometry/TriangleMesh/Processing/Decimation.h"
#include "Geometry/Structure/PointCloud.h"

using namespace dragon;

void ReadCenterLines(const std::string &path, geometry::Point3List &centers, geometry::ScalarList &radius)
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
        std::cout << "Usage: CenterLine2SurfacePoints [input_filename] [output_filename] [voxel_resolution = 0.25] [radius_factor = -1.0]"<<std::endl;
        return 0;        
    }

    geometry::TriangleMesh generated_mesh;
    geometry::Point3List centers;
    geometry::ScalarList radius;
    ReadCenterLines(argv[1], centers, radius);
    std::string output_filename = argv[2];
    float voxel_resolution = 0.25;
    float radius_factor = -1.0;
    if(argc > 3)
    voxel_resolution = std::atof(argv[3]);
    if(argc > 4)
    radius_factor = std::atof(argv[4]);

    // // pre-processing: voxel clustering
    // if(voxel_resolution > 0)
    // {
    //     auto clusters = geometry::VoxelClustering(centers, voxel_resolution * 2);
    //     geometry::Point3List ccenters;
    //     geometry::ScalarList cradius;
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
	geometry::ScalarList cradius;
        auto clusters = geometry::RadiusClustering(centers, radius, radius_factor);
	for (auto &c: clusters)
	{
            geometry::Point3 tmp_center = geometry::Point3::Zero();
            double tmp_r = 0;
            for(auto &id: c)
            {
                tmp_center += centers[id];
                tmp_r += radius[id];
            }
            ccenters.push_back(tmp_center /c.size());
            cradius.push_back(tmp_r / c.size());
        }
        centers = ccenters;
        radius = cradius;    
        std::cout<<"After clustering: "<<centers.size()<<std::endl;
    }      
    
    geometry::PointCloud surface_pcd = reconstruction::Centerline2SurfacePoints(centers, radius, 5);
    std::cout<<surface_pcd.points.size()<<std::endl;
    surface_pcd.VoxelClustering(voxel_resolution);
    surface_pcd.WriteToFile(output_filename);
    
    std::cout<<surface_pcd.points.size()<<std::endl;
    return 0;
}
