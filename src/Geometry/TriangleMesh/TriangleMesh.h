#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H
#include "IO/ConsoleColor.h"
#include "Geometry/BasicGeometry.h"
// #include "Geometry/PointCloud.h"
#include <memory>
#include "Geometry/Structure/BoundingBox.h"
#include "Geometry/Structure/PointCloud.h"
namespace dragon
{
namespace geometry
{
    class TriangleMesh
    {
        public:
        bool LoadFromPLY(const std::string &filename);
        bool LoadFromOBJ(const std::string &filename);
        bool LoadFromFile(const std::string & filename);
        void ComputeNormals();
        void Transform(const TransformationMatrix & T);
        bool HasColors() const
        {
            return colors.size() == points.size() && colors.size() > 0;
        }
        void Reset()
        {
            triangles.clear();
            points.clear();
            normals.clear();
            colors.clear();
        }
        void LoadFromMeshes(const std::vector<TriangleMesh > &meshes);
        bool HasNormals() const
        {
            return normals.size() == points.size() && normals.size() > 0;
        }
        // std::shared_ptr<TriangleMesh> QuadricSimplify(size_t target_num) const;
        // std::shared_ptr<TriangleMesh> ClusteringSimplify(float grid_len) const;
        // std::shared_ptr<PointCloud> GetPointCloud() const;
        std::shared_ptr<PointCloud> GetPointCloud() const
        {
            PointCloud p;
            p.points = points;
            p.normals = normals;
            p.colors = colors;
            return std::make_shared<PointCloud>(p);
        }
        void Scale(double scale)
        {
            for(size_t i = 0; i != points.size(); ++i)
            points[i] *= scale;
        }
        void FlipNormal()
        {
            for(size_t i = 0; i != triangles.size(); ++i)
            triangles[i].reverseInPlace();
        }
        void Normalize()
        {
            points = geometry::Normalize(points);
        }
        size_t GetPointSize() const{return points.size();}
        size_t GetTriangleSize() const{return triangles.size();}
        bool WriteToPLY(const std::string &filename) const;
        bool WriteToOBJ(const std::string &filename) const;
        BoundingBox GetBoundingBox() const;
        Point3uiList triangles;
        Point3List points;
        Point3List normals;
        Point3List colors; 
        
    };
}
}

#endif