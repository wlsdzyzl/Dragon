#include "TriangleMesh.h"
#include "IO/PLYManager.h"
#include "IO/OBJManager.h"
#include <iostream>
#include <fstream>
#include "./Processing/Simplification.h"
#include "Tool/CppExtension.h"
namespace dragon
{
namespace geometry
{
namespace mesh
{
    bool TriangleMesh::LoadFromPLY(const std::string &filename)
    {
        Reset();
        std::vector<io::AdditionalElement> additional_labels;
        return io::ReadPLY(filename,points,normals,colors,triangles, additional_labels);
    }
    bool TriangleMesh::LoadFromOBJ(const std::string& filename) 
    {
        Reset();
        return io::ReadOBJ(filename,points,normals,colors,triangles);
    }
    bool TriangleMesh::LoadFromFile(const std::string & filename)
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
    void TriangleMesh::Transform(const TransformationMatrix & T)
    {
        TransformPoints(T,points);
        if(HasNormals())
        TransformNormals(T,normals);
    }
    // std::shared_ptr<TriangleMesh> TriangleMesh::QuadricSimplify(size_t target_num) const
    // {
    //     TriangleMesh s_mesh = *this;
    //     QuadricSimplification(s_mesh,target_num);
    //     return std::make_shared<TriangleMesh>(s_mesh);
    // }
    // std::shared_ptr<TriangleMesh> TriangleMesh::ClusteringSimplify(float grid_len) const
    // {
    //     TriangleMesh s_mesh = *this;
    //     ClusteringSimplification(s_mesh,grid_len);
    //     return std::make_shared<TriangleMesh>(s_mesh);
    // }
    // std::shared_ptr<TriangleMesh> TriangleMesh::Prune(size_t min_points) const
    // {
    //     TriangleMesh s_mesh = *this;
    //     MeshPruning(s_mesh,min_points);
    //     return std::make_shared<TriangleMesh>(s_mesh);
    // }
    // std::shared_ptr<PointCloud> TriangleMesh::GetPointCloud() const
    // {
    //     PointCloud pcd;
    //     pcd.normals = normals;
    //     pcd.points = points;
    //     pcd.colors = colors;
    //     return std::make_shared<PointCloud>(pcd);
    // }
    void TriangleMesh::LoadFromMeshes(const std::vector<TriangleMesh > &meshes)
    {
        Reset();
        int start_index = 0;
        size_t triangles_num;
        for(size_t i = 0; i < meshes.size(); ++i)
        {
            triangles.insert(triangles.end(),meshes[i].triangles.begin(),meshes[i].triangles.end());
            colors.insert(colors.end(),meshes[i].colors.begin(),meshes[i].colors.end());
            points.insert(points.end(),meshes[i].points.begin(),meshes[i].points.end());
            normals.insert(normals.end(),meshes[i].normals.begin(),meshes[i].normals.end());
            triangles_num = meshes[i].triangles.size();
            if(start_index != 0)
            for(size_t i = triangles.size()-1, j = 0;j!=triangles_num; ++j, --i )
            {
                triangles[i](0) = triangles[i](0) + start_index;
                triangles[i](1) = triangles[i](1) + start_index;
                triangles[i](2) = triangles[i](2) + start_index;
            }
            start_index += meshes[i].points.size();
        }
    }
    void TriangleMesh::ComputeNormals()
    {
        std::vector<Reference> references;
        references.resize(points.size());
        UpdateReferences(triangles,references);
        normals.resize(points.size());
        Point3List triangle_normals;
        triangle_normals.resize(triangles.size());
        Point3 n,p1,p2,p3;
      
        for(size_t i = 0;i!= triangles.size(); ++i)
        {
            p1 = points[triangles[i](0)];
            p2 = points[triangles[i](1)];
            p3 = points[triangles[i](2)];
            n = ((p2 - p1).cross(p3 - p1));
            n.normalize();
            triangle_normals[i] = n;
        }
		for(size_t i = 0;i!=points.size();++i)
		{

			Point3 vnormal; 
            vnormal.setZero();
            for(size_t j = 0;j!=references[i].size();++j)
			{
				vnormal += triangle_normals[references[i][j].first];
			}
			vnormal.normalize();

            normals[i] = vnormal;
		}
    }
    BoundingBox TriangleMesh::GetBoundingBox() const
    {
        BoundingBox bb;
        for(size_t i = 0; i != points.size(); ++i)
        {
            bb.AddPoint(points[i]);
        }
        return bb;
    }
    bool TriangleMesh::WriteToPLY(const std::string &filename) const
    {
        return io::WritePLY(filename, points, normals, colors, triangles);
    }
    bool TriangleMesh::WriteToOBJ(const std::string& filename) const
    {
        return io::WriteOBJ(filename,points,normals,colors,triangles);
    }
}
}
}