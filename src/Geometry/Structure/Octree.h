#ifndef DRAGON_OCTREE_H
#define DRAGON_OCTREE_H
#include "Geometry/BasicGeometry.h"
#include "Geometry/Structure/PointCloud.h"
namespace dragon
{
namespace geometry
{
    class OctreeNode
    {
        public:
        bool is_leaf = true;
        double weight;
        bool InNode(const Point3 &p)
        {
            double half_width = width / 2;
            return (p(0) >= center(0) - half_width) && (p(0) <= center(0) + half_width )
                    && (p(1) >= center(1) - half_width ) && (p(1) <= center(1) + half_width)
                    && (p(2) >= center(2) - half_width ) && (p(2) <= center(2) + half_width);
        }
        void Split(int max_depth)
        {
            if(is_leaf == true && depth < max_depth )
            {
                // split the nodes
                nodes = new OctreeNode[8];
                double next_half_width = width / 4;
                double next_width = width / 2;
                int next_depth = depth + 1;
                for(int i = 0; i != 8; ++i)
                {
                    nodes[i].width = next_width;
                    nodes[i].depth = next_depth;
                    nodes[i].parent = this;
                    
                    if(i & 1)
                    nodes[i].center(0) = center(0) + next_half_width;
                    else
                    nodes[i].center(0) = center(0) - next_half_width;
                    if((i >> 1) & 1)
                    nodes[i].center(1) = center(1) + next_half_width;
                    else
                    nodes[i].center(1) = center(1) - next_half_width;
                    if((i >> 2) & 1)
                    nodes[i].center(2) = center(2) + next_half_width;
                    else
                    nodes[i].center(2) = center(2) - next_half_width;
                } 
                for(int i = 0; i != 8; ++i)
                nodes[i].Split(max_depth);
                is_leaf = false;  
            }            
        }
        void AddPoint(const Point3List &points, int pid, int max_depth)
        {
            
            if(is_leaf == false)
            {
                // std::cout<<"not leaf"<<std::endl;
                int i;
                for(i = 0; i != 8; ++i)
                {
                    // std::cout<<i<<" "<<depth<<" "<<nodes[i].width<<std::endl;
                    // std::cout<<"center: "<<nodes[i].center.transpose()<<std::endl;
                    // std::cout<<(nodes[i].center + geometry::Point3(nodes[i].width / 2, nodes[i].width / 2, nodes[i].width / 2)).transpose()<<" "<<
                    //     (nodes[i].center - geometry::Point3(nodes[i].width / 2, nodes[i].width / 2, nodes[i].width / 2)).transpose()<<" / "<<points[pid].transpose()<<std::endl;
                    if(nodes[i].InNode(points[pid]))
                    {

                        nodes[i].AddPoint(points, pid, max_depth);
                        break;
                    }
                }
                // if(i == 8)
                // exit(0);
            }
            else 
            {
                pidlist.push_back(pid);
                // the point is too far away from the center, or the points is much than 2
                // std::cout<<(points[pid] - center).norm()<<" "<<width * 0.866 /  std::pow(2, max_depth)<<std::endl;
                // || ((points[pid] - center).norm() > width * 0.866 /  std::pow(2, max_depth))
                if(depth < max_depth && (pidlist.size() > 0  ))
                {
                    // split the nodes
                    nodes = new OctreeNode[8];
                    double next_half_width = width / 4;
                    double next_width = width / 2;
                    int next_depth = depth + 1;
                    for(int i = 0; i != 8; ++i)
                    {
                        nodes[i].width = next_width;
                        nodes[i].depth = next_depth;
                        nodes[i].parent = this;
                        
                        if(i & 1)
                        nodes[i].center(0) = center(0) + next_half_width;
                        else
                        nodes[i].center(0) = center(0) - next_half_width;
                        if((i >> 1) & 1)
                        nodes[i].center(1) = center(1) + next_half_width;
                        else
                        nodes[i].center(1) = center(1) - next_half_width;
                        if((i >> 2) & 1)
                        nodes[i].center(2) = center(2) + next_half_width;
                        else
                        nodes[i].center(2) = center(2) - next_half_width;
                    }
                    for(size_t id = 0; id != pidlist.size(); ++id)
                    {
                        for(int i = 0; i != 8; ++i)
                        {

                            if(nodes[i].InNode(points[pidlist[id]]))
                            {
                                nodes[i].AddPoint(points, pidlist[id], max_depth);
                                break;
                            }
                        }
                    }
                    // pidlist.clear();
                    is_leaf = false;     
                }
            }

        }
        void Reset()
        {
            pidlist.clear();
            if(!is_leaf)
            {
                for(int i = 0; i != 8; ++i)
                {
                    nodes[i].Reset();
                }
                delete[] nodes;
                is_leaf = true;
            }
            depth = 0;
            width = 0;

        }
        void SetNodeIDAndGetAllNodes(std::vector<std::vector<OctreeNode *>> &all_nodes)
        {
            id = all_nodes[depth].size();
            all_nodes[depth].push_back(this);
            if(!is_leaf)
            {
                for(int i = 0; i != 8; ++i)
                {
                    nodes[i].SetNodeIDAndGetAllNodes(all_nodes);
                }
            }
        }
        void GetAllLeaves(std::vector<OctreeNode *> &leaves)
        {
            if(is_leaf)
            {
                leaves.push_back(this);
            }
            else
            {
                for(int i = 0; i != 8; ++i)
                nodes[i].GetAllLeaves(leaves);
            }            
        }
        OctreeNode *LocateLeaf(const geometry::Point3 &p)
        {
            if(is_leaf) return this;
            for(int i = 0; i != 8; ++i)
            {
                if(nodes[i].InNode(p)) return nodes[i].LocateLeaf(p);       
            }
            return nullptr;
        }
        OctreeNode *Locate(const geometry::Point3 &p, int d)
        {
            if(depth == d) return this;
            else if( !is_leaf)
            {
                for(int i = 0; i != 8; ++i)
                {
                    if(nodes[i].InNode(p)) return nodes[i].Locate(p, d);       
                }
            }
            std::cout<<"What happened ??"<<std::endl;
            return nullptr;
        }
        OctreeNode *LocateAndSplit(const geometry::Point3 &p, int d)
        {
            if(depth == d) return this;
            if(is_leaf)
            {
                // split the nodes
                nodes = new OctreeNode[8];
                double next_half_width = width / 4;
                double next_width = width / 2;
                int next_depth = depth + 1;
                for(int i = 0; i != 8; ++i)
                {
                    nodes[i].width = next_width;
                    nodes[i].depth = next_depth;
                    nodes[i].parent = this;
                    
                    if(i & 1)
                    nodes[i].center(0) = center(0) + next_half_width;
                    else
                    nodes[i].center(0) = center(0) - next_half_width;
                    if((i >> 1) & 1)
                    nodes[i].center(1) = center(1) + next_half_width;
                    else
                    nodes[i].center(1) = center(1) - next_half_width;
                    if((i >> 2) & 1)
                    nodes[i].center(2) = center(2) + next_half_width;
                    else
                    nodes[i].center(2) = center(2) - next_half_width;
                } 
                is_leaf = false;  
            }
            for(int i = 0; i != 8; ++i)
            {
                if(nodes[i].InNode(p)) return nodes[i].LocateAndSplit(p, d);       
            }
            std::cout<<"What happened ??"<<std::endl;
            return nullptr;
        }
        ~OctreeNode()
        {
            Reset();
        }
        OctreeNode *nodes = nullptr;
        OctreeNode * parent = nullptr;
        Point3 center;
        Point3 normal = Point3::Zero();
        // point id fall into this node
        std::vector<int> pidlist;
        int depth = 0;
        int id = -1;
        double width;
    };
    class Octree
    {
        public:
        Octree() = default;
        Octree(int max_d):max_depth(max_d){}
        void BuildTree(const geometry::PointCloud &pcd);

        OctreeNode head;
        int max_depth = 6;
        int node_size = 0;
        geometry::PointCloud sampled_pcd;
        std::vector<OctreeNode *> point_to_leaf;
        std::vector<OctreeNode *> all_leaves;
        std::vector< std::vector<OctreeNode *>> all_nodes;
        void Reset()
        {
            head.Reset();
            sampled_pcd.Reset();
            point_to_leaf.clear();
            all_nodes.clear();
            all_leaves.clear();
        }
        void UniformSplit()
        {
            head.Split(max_depth);
        }
        OctreeNode* LocateLeaf(const Point3 &p)
        {
            if(head.InNode(p))
            return head.LocateLeaf(p);
            else return nullptr;
        }
        OctreeNode* Locate(const Point3 &p, int depth)
        {
            if(head.InNode(p))
            return head.Locate(p, depth);
            else return nullptr;
        }
        OctreeNode* LocateAndSplit(const Point3 &p, int depth)
        {
            if(head.InNode(p))
            return head.LocateAndSplit(p, depth);
            else return nullptr;
        }
        void SetNodeIDAndGetAllNodes()
        {
            all_nodes.clear();
            all_nodes.resize(max_depth + 1);
            head.SetNodeIDAndGetAllNodes(all_nodes);
        }
        std::shared_ptr<geometry::PointCloud> GetPointCloud() const;
    };
}
}
#endif