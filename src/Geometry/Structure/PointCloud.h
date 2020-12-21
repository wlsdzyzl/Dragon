#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H
#include "Geometry/BasicGeometry.h"
#include <Eigen/Core>
#include <memory>
namespace dragon
{
namespace geometry 
{

    class RGBDFrame;
    class PointCloud
    {
        public:
            PointCloud()=default; 
            size_t GetSize() const
            {
                return points.size();
            }
            bool HasColors() const
            {
                return colors.size() == points.size() && colors.size() > 0;
            }

            bool HasNormals() const
            {
                return normals.size() == points.size() && normals.size() > 0;
            }
            bool LoadFromPLY(const std::string &filename);
            bool LoadFromOBJ(const std::string &filename);
            bool LoadFromFile(const std::string & filename);
            bool WriteToOBJ(const std::string &filename);
            void EstimateNormals(float radius = 0.1, int knn = 30);
            void Transform(const geometry::Matrix4 &T);
            std::shared_ptr<PointCloud> DownSample(float grid_len) const;
            bool WriteToPLY(const std::string &fileName) const;
            void MergePCD(const PointCloud & another_pcd);
            void Reset()
            {
                points.clear();
                normals.clear();
                colors.clear();
            }
            
            geometry::Point3List points;
            geometry::Point3List normals;
            geometry::Point3List colors;
    };

    typedef std::shared_ptr<PointCloud> PointCloudPtr;
};
}
#endif