#ifndef DRAGON_MESH_TO_SDF_H
#define DRAGON_MESH_TO_SDF_H
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include "CubeHandler.h"
#include "Geometry/Structure/KDTree.h"
#include "Geometry/Structure/HalfEdge.h"
#include "Geometry/Structure/PointCloud.h"
#include "Geometry/Structure/Graph.h"
// Naive way to generate sdf from triangle mesh.
// for each voxel, use kdtree to find the closest vertex.
// then, we compute the distances from the voxel to the vertex-connected faces, the final distance will be the minumum.
// The sign of distance will be determined by the face normal.
namespace dragon
{
namespace reconstruction
{

    void Mesh2SDF(const geometry::TriangleMesh &mesh, CubeHandler &cube_handler, float voxel_resolution = 0.01);
    std::vector<double> Mesh2SDF(const geometry::TriangleMesh &mesh, const geometry::Point3List & points);
    // std::vector<double> Mesh2SDF(const geometry::TriangleMesh &mesh, const geometry::Point3i & grid_num, geometry::Point3 origin, float voxel_resolution)
    std::vector<double> Mesh2SDF(const geometry::TriangleMesh &mesh, const geometry::Point3i & grid_num, geometry::Point3 origin = geometry::Point3::Zero(), float voxel_resolution = 0.01);
    std::vector<double> Mesh2Indicator(const geometry::TriangleMesh &mesh, const geometry::Point3i & grid_num, geometry::Point3 origin = geometry::Point3::Zero(), float voxel_resolution = 0.01, double truncation = 1e6, bool coarse = true);
    std::vector<double> NormalPCD2Indicator(const geometry::PointCloud &pcd, const geometry::Point3i & grid_num, geometry::Point3 origin = geometry::Point3::Zero(), float voxel_resolution = 0.01);

    void CenterLine2SDF(const geometry::Point3List &centers, const std::vector<double> &radius, CubeHandler &cube_handler, float voxel_resolution = 0.01, bool ordered = true, int knn = 2, bool fast_computation = true);
    geometry::PointCloud Centerline2SurfacePoints(const geometry::Point3List &centers, const std::vector<double> &radius, size_t n_points = 10);
    void SkeletonGraph2SDF(const geometry::SkeletonGraph &sgraph, 
        CubeHandler &cube_handler, 
        float voxel_resolution, 
        int knn,
        bool fast_computation);
}
}
#endif