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
        std::cout << "Usage: CenterLineAgreeMesh [input_xyzr] [input_mesh]"<<std::endl;
        return 0;        
    }

    geometry::TriangleMesh generated_mesh;
    geometry::Point3List centers;
    std::vector<double> radius;
    ReadCenterLines(argv[1], centers, radius);
    generated_mesh.LoadFromFile(argv[2]);
    // double radius_factor = 0.75;
    // // pre-processing: radius clustering
    // if(radius_factor > 0)
    // {
    //     geometry::Point3List ccenters;
    //     std::vector<double> cradius;
    //     auto clusters = geometry::RadiusClustering(centers, radius, radius_factor);
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

    std::vector<double> sdfs = reconstruction::Mesh2SDF(generated_mesh, centers);
    int insider_count = 0;
    double radius_diff = 0;
    int valid_count = 0;
    for(size_t i = 0; i != sdfs.size(); ++i)
    {
        if(radius[i] > 1)
        {
            if(sdfs[i] <= 0.0) 
            {
                insider_count += 1;
                radius_diff += std::abs((sdfs[i] + radius[i]));
            }
            valid_count += 1;
        }
    }
    std::cout << (insider_count + 0.0) / valid_count << std::endl;
    std::cout << radius_diff / insider_count << std::endl;
    return 0;
}