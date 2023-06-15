#include "PointCloud.h"
#include "Geometry/Structure/KDTree.h"
#include "IO/ConsoleColor.h"
#include "IO/PLYManager.h"
#include "IO/OBJManager.h"
#include "Tool/CppExtension.h"
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <omp.h>
#include <queue>
namespace dragon
{
namespace geometry 
{
    struct PCDEdge
    {
        PCDEdge() = default;
        PCDEdge(size_t _v1, size_t _v2, double _d):v1(_v1), v2(_v2), dist(_d){}
        size_t v1;
        size_t v2;
        double dist;
    };
    struct PCDEdgeGreater
    {
        bool operator()(const PCDEdge &e1, const PCDEdge &e2 )
        {
            return e1.dist > e2.dist;
        }
    };
    void PointCloud::MergePCD(const PointCloud & another_pcd)
    {
        int merged_point_size = points.size() + another_pcd.points.size();
        int merged_colors_size = colors.size() + another_pcd.colors.size();
        int merged_normals_size = normals.size() + another_pcd.normals.size();
        if(merged_point_size != merged_colors_size && merged_colors_size > 0)
        {
            std::cout<<RED<<"[Error]::[MergePCD]::The color are not matching."<<RESET<<std::endl;
            return;
        }
        if(merged_point_size != merged_normals_size && merged_normals_size > 0)
        {
            std::cout<<RED<<"[Error]::[MergePCD]::The normal are not matching."<<RESET<<std::endl;
            return;
        }
        points.insert(points.end(), another_pcd.points.begin(), another_pcd.points.end());
        colors.insert(colors.end(), another_pcd.colors.begin(), another_pcd.colors.end());
        normals.insert(normals.end(), another_pcd.normals.begin(), another_pcd.normals.end());
    }
    void PointCloud::VoxelClustering(double grid_len)
    {
        auto clusters = geometry::VoxelClustering(points, grid_len);
        geometry::Point3List cpoints;
        geometry::Point3List cnormals;
        geometry::Point3List ccolors;
        for(auto &c: clusters)
        {
            geometry::Point3 tmp_point = geometry::Point3::Zero();
            geometry::Point3 tmp_normal = geometry::Point3::Zero();
            geometry::Point3 tmp_color = geometry::Point3::Zero();
            for(auto &id: c)
            tmp_point += points[id];
            cpoints.push_back(tmp_point / c.size());
            if(HasNormals())
            {
                for(auto &id: c)
                tmp_normal += normals[id];
                cnormals.push_back(tmp_normal / c.size());               
            }
            if(HasColors())
            {
                for(auto &id: c)
                tmp_color += colors[id];
                ccolors.push_back(tmp_color / c.size());
            }
        }
        points = cpoints;
        normals = cnormals;
        colors = ccolors;
    }
    void PointCloud::EstimateNormals(float radius, int knn)
    {   
        normals.resize(points.size());        

        geometry::KDTree<> kdtree;
        kdtree.BuildTree(points);
        std::cout<<BLUE<<"[EstimateNormals]::[INFO]::RadiusSearch "<<knn<<" nearest points, radius: "<<radius<<RESET<<std::endl;
        std::vector<std::vector<PCDEdge>> vertex_to_edge(points.size());
        for(size_t i = 0; i < points.size(); ++i)
        {
            
            std::vector<int> indices; 
            std::vector<float> dists; 
            
            // kdtree.KnnSearch(points[i], indices, dists, 
            //     knn, geometry::SearchParameter(1024));
            kdtree.KnnRadiusSearch(points[i], indices, dists,
                knn, radius, geometry::SearchParameter(1024));
            // kdtree.RadiusSearch(points[i], indices, dists, radius, 
            //     knn, geometry::SearchParameter(1024));            
            // std::cout<<dists.size()<<std::endl;
            geometry::Point3List nearest_points;
            for(size_t j = 0; j!= indices.size();++j)
            {
                
                nearest_points.push_back(points[indices[j]]);
                vertex_to_edge[i].push_back(PCDEdge(i, indices[j], dists[j]));
                // vertex_to_edge[indices[j]].push_back(PCDEdge(indices[j], i, dists[j]));
                //std::cout <<indices[j]<<" "<<std::endl;
            }
            
            //std::cout <<std::endl;
            auto result = FitPlane(nearest_points);
            normals[i] = std::get<0>(result);
        }

#if 1
        // for(size_t i = 0; i != vertex_to_edge.size(); ++i)
        // std::cout<<vertex_to_edge[i].size()<<" ";
        // minimal spanning tree to determine the orientation of normal
        // flip normals
        std::priority_queue<PCDEdge, std::vector<PCDEdge>, PCDEdgeGreater> edges_q;
        std::unordered_set<size_t> visited;
        for(size_t i = 0; i != vertex_to_edge[0].size(); ++i)
        edges_q.push(vertex_to_edge[0][i]);


        visited.insert(0);
        size_t iter = 0;
        while(visited.size() != points.size() && !edges_q.empty())
        {
            auto top = edges_q.top();
            edges_q.pop();
            // std::cout<<"flip normal-----"<<top.v1<<" "<<top.v2<<std::endl;
            iter ++;
            // if(iter == 10000) break;
            if(visited.find(top.v2) != visited.end())
            continue;
            
            if(normals[top.v2].dot(normals[top.v1]) < 0)
            {
                // std::cout<<"flip normal"<<std::endl;
                normals[top.v2] = - normals[top.v2];
            }
            visited.insert(top.v2);
            
            for(size_t i = 0; i != vertex_to_edge[top.v2].size(); ++i)
            {
                edges_q.push(vertex_to_edge[top.v2][i]);
                // visited.insert(vertex_to_edge[top.v2][i].v2);
            }
            
        }
        for(size_t i = 0; i != normals.size(); ++i)
            if(visited.find(i) == visited.end())
            normals[i].setZero();
#endif 
    }
    std::shared_ptr<PointCloud> PointCloud::DownSample(float grid_len) const
    {
        PointCloud pcd = *this;
        std::unordered_map<Eigen::Vector3i, std::pair<int, int>, geometry::VoxelGridHasher > grids;     
         
        int ptr = 0;  
        bool has_color = HasColors();
        bool has_normal = HasNormals();
        //float half_len = grid_len / 2;
        for(size_t i = 0;i!= pcd.points.size(); ++i)
        {
            Eigen::Vector3i index = Eigen::Vector3i(std::floor(pcd.points[i](0) / grid_len), 
                std::floor(pcd.points[i](1) / grid_len), std::floor(pcd.points[i](2) / grid_len));
            if(grids.find(index) == grids.end())
            {
                pcd.points[ptr] = pcd.points[i];
                if(has_color)
                pcd.colors[ptr] = pcd.colors[i];
                if(has_normal)
                pcd.normals[ptr] = pcd.normals[i];
                grids[index] = std::make_pair(ptr, 1);
                ptr++;
            }
            else
            {
                pcd.points[grids[index].first] += pcd.points[i];
                if(has_color)
                pcd.colors[grids[index].first] += pcd.colors[i];
                if(has_normal)
                pcd.normals[grids[index].first] += pcd.normals[i];
                grids[index].second += 1;
            }
        }
        for(auto iter = grids.begin(); iter != grids.end(); ++iter)
        {
            pcd.points[iter->second.first] /= iter->second.second;
            if(has_color) pcd.colors[iter->second.first] /= iter->second.second;
            if(has_normal) pcd.normals[iter->second.first] /= iter->second.second;            
        }
        pcd.points.resize(ptr);
        if(has_color) pcd.colors.resize(ptr);
        if(has_normal) pcd.normals.resize(ptr);
        //if(HasNormals()) pcd.EstimateNormals();
        return std::make_shared<PointCloud>(pcd);
    }
    bool PointCloud::LoadFromPLY(const std::string &filename)
    {
        Reset();
        std::vector<io::AdditionalElement> additional_labels;
        geometry::Point3uiList triangles;
        return io::ReadPLY(filename,points,normals,colors, triangles, additional_labels);
    }
    bool PointCloud::LoadFromOBJ(const std::string &filename)
    {
        Reset();
        return io::ReadOBJ(filename,points,normals,colors);
    }
    bool PointCloud::LoadFromFile(const std::string & filename)
    {
        std::vector<std::string> result = tool::RSplit(filename, ".", 1);
        if(result.size() == 2)
        {
            if(result[1] == "obj")//obj
            {
                return LoadFromOBJ(filename);
            }
            else if(result[1] == "ply")
            {
                return LoadFromPLY(filename);
            }
        }
        std::cout<<YELLOW<<"[WARNING]::[LoadFromFile]::Dragon only supports obj and ply file."<<RESET<<std::endl;
        return false;
    }
    bool PointCloud::WriteToOBJ(const std::string &filename)
    {
        return io::WriteOBJ(filename,points,normals,colors);
    }
    void PointCloud::Transform(const TransformationMatrix &T)
    {
        points = geometry::TransformPoints(T,points);
        if(HasNormals())
        normals = geometry::TransformNormals(T,normals);
    }

    bool PointCloud::WriteToPLY(const std::string &filename) const
    {
        return io::WritePLY(filename,points,normals,colors);
    }
}
}