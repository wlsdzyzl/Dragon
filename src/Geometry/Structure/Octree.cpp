#include "Octree.h"
#include "Geometry/Structure/BoundingBox.h"
namespace dragon
{
namespace geometry
{
    void Octree::BuildTree(const geometry::PointCloud &pcd_)
    {
        pcd = pcd_;
        BoundingBox bb = pcd.GetBoundingBox();
        auto &points = pcd.points;
        head.width = std::max(bb.x_max - bb.x_min, std::max(bb.y_max - bb.y_min, 
            bb.z_max - bb.z_min)) * 1.5;
        head.center = Point3(bb.x_max + bb.x_min, bb.y_max + bb.y_min, bb.z_max + bb.z_min) / 2.0;
        std::cout<<"head: "<<head.center.transpose()<<" "<<head.width<<std::endl;
        std::cout<<"BB: "<<bb.y_max<<" "<<bb.y_min<<std::endl;
        point_to_leaf.clear();
        point_to_leaf.resize(points.size(), nullptr);
        for(size_t i = 0; i != points.size(); ++i)
        {
            // std::cout<<"p "<<i<<std::endl;
            head.AddPoint(points, i, max_depth, point_to_leaf);
        }
        head.SetLeafIDAndGetAllLeaves(all_leaves);
    }
    std::shared_ptr<geometry::PointCloud> Octree::GetPointCloud() const
    {
        geometry::PointCloud pcd;
        for(size_t i = 0; i != all_leaves.size(); ++i)
        {
            pcd.points.push_back(all_leaves[i]->center);
            pcd.normals.push_back(all_leaves[i]->normal);
        }
        return std::make_shared<geometry::PointCloud>(pcd);
    }
}
}