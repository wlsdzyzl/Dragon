#include "RBF.h"
namespace dragon
{
namespace reconstruction
{
    void GenerateSamplePoints(const geometry::PointCloud &pcd, geometry::Vec4List &samp)
    {
        auto &points = pcd.points;
        auto &normals = pcd.normals;
        samp.clear();
        for(size_t i = 0; i != points.size(); ++i)
        {
            geometry::Vector4 sample_p0;
            geometry::Vector4 sample_p1;
            geometry::Vector4 sample_p2;
            sample_p0.head<3>() = points[i];
            sample_p0(3) = 0;
            sample_p1.head<3>() = points[i] + normals[i];
            sample_p1(3) = 1.0;
            sample_p2.head<3>() = points[i] - normals[i];
            sample_p2(3) = -1.0;
            samp.push_back(sample_p0);
            samp.push_back(sample_p1);
            samp.push_back(sample_p2);
        }
    }
    std::shared_ptr<geometry::TriangleMesh> RBF(const geometry::PointCloud &pcd, CubeHandler &cube_handler)
    {
        
        // use sample points to construct normal equation
        auto &points = pcd.points;
        // auto &normals = pcd.normals;
        if(!pcd.HasNormals())
        {
            std::cout<<RED<<"[ERROR]::[RBF]::RBF is not responsible for normal estimation."<<RESET<<std::endl;
            return std::shared_ptr<geometry::TriangleMesh>();
        }
        geometry::Vec4List samp;
        GenerateSamplePoints(pcd, samp);
        geometry::MatrixX A(samp.size(), samp.size());
        geometry::VectorX b(samp.size());
        geometry::VectorX x(samp.size());
        for(size_t i = 0; i != samp.size(); ++i)
        {
            for(size_t j = 0; j != samp.size(); ++j)
            {
                A(i, j) = std::pow((samp[i].head<3>() - samp[j].head<3>()).norm(), 3);
            }
            b(i) = samp[i](3);
        }
        // geometry::MatrixX A(points.size(), points.size());
        // A.setOnes();
        // geometry::VectorX b(points.size());
        // geometry::VectorX x(points.size());
        // for(size_t i = 0; i != points.size(); ++i)
        // {
        //     geometry::Vector3 sample_p1 = points[i] + normals[i];
        //     geometry::Vector3 sample_p2 = points[i] - normals[i];
        //     for(size_t j = 0; j != points.size(); ++j)
        //     {
        //         A(i, j) = pow((points[i] - points[j]).norm(), 3);
        //     }
        //     b(i) = 0;

        // }
        std::cout<<BLUE<<"[INFO]::[RBF]::Construct rbf normal equation."<<RESET<<std::endl;
        // std::cout<<A<<"\n----------------------"<<std::endl;
        // std::cout<<b<<std::endl;
        // std::cout<<A.inverse()<<"\n----------------------"<<std::endl;
        // solve rbf function
        x = geometry::SolveBySVD(A, b);
        std::cout<<BLUE<<"[INFO]::[RBF]::Solve equation."<<RESET<<std::endl;
        // generate voxel center distances
        // std::cout<<x<<std::endl;
        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
            geometry::VectorX w(samp.size());
            for(size_t i = 0; i != samp.size(); ++i)
            w(i) = pow((p - samp[i].head<3>()).norm(), 3);
            return w.dot(x);
            };
        cube_handler.IntegratePoints(points, get_sdf);
        std::cout<<BLUE<<"[INFO]::[RBF]::Finish points integration, extracting mesh ..."<<RESET<<std::endl;
        geometry::TriangleMesh mesh;
        cube_handler.ExtractTriangleMesh(mesh);
        return std::make_shared<geometry::TriangleMesh>(mesh);
    }
}
}