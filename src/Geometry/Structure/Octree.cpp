#include "Octree.h"
#include "Geometry/Structure/BoundingBox.h"
namespace dragon
{
namespace geometry
{
    void Octree::BuildTree(const geometry::PointCloud &pcd)
    {
        
        BoundingBox bb = pcd.GetBoundingBox();
        auto &points = pcd.points;
        // auto &normals = pcd.normals;
        head.width = std::max(bb.x_max - bb.x_min, std::max(bb.y_max - bb.y_min, 
            bb.z_max - bb.z_min)) * 1.5;
        head.center = Point3(bb.x_max + bb.x_min, bb.y_max + bb.y_min, bb.z_max + bb.z_min) / 2.0;
        std::cout<<"head: "<<head.center.transpose()<<" "<<head.width<<std::endl;
        std::cout<<"BB: "<<bb.y_max<<" "<<bb.y_min<<std::endl;
        all_nodes.clear();
        all_nodes.resize(max_depth + 1);
        for(size_t i = 0; i != points.size(); ++i)
        {
            // std::cout<<"p "<<i<<std::endl;
            head.AddPoint(points, i, max_depth);
        }
        SetNodeIDAndGetAllNodes();
        // head.SetLeafIDAndGetAllLeaves(all_leaves);
        head.GetAllLeaves(all_leaves);
        sampled_pcd.Reset();
        point_to_leaf.clear();
    }
    std::shared_ptr<geometry::PointCloud> Octree::GetPointCloud() const
    {
        geometry::PointCloud pcd;
        for(size_t i = 0; i != all_nodes.back().size(); ++i)
        {
            if(all_nodes.back()[i]->normal.norm() > 0)
            {
                pcd.points.push_back(all_nodes.back()[i]->center);
                pcd.normals.push_back(all_nodes.back()[i]->normal.normalized());
            }
        }
        return std::make_shared<geometry::PointCloud>(pcd);
    }
}
}