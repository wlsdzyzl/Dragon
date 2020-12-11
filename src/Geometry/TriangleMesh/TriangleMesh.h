#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H
#include "IO/ConsoleColor.h"
#include "Geometry/BasicGeometry.h"
// #include "Geometry/PointCloud.h"
#include <memory>
#include "../BoundingBox.h"
namespace dragon
{
namespace geometry
{
namespace mesh
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
        // std::shared_ptr<TriangleMesh> Prune(size_t min_points) const;
        // std::shared_ptr<PointCloud> GetPointCloud() const;
        size_t GetPointSize() const{return points.size();}
        size_t GetTriangleSize() const{return triangles.size();}
        bool WriteToPLY(const std::string &fileName) const;
        bool WriteToOBJ(const std::string &fileName) const;
        BoundingBox GetBoundingBox() const;
        Point3uiList triangles;
        Point3List points;
        Point3List normals;
        Point3List colors; 
        
    };
}
}
}

#endif