#ifndef DRAGON_MESH_TO_SDF_H
#define DRAGON_MESH_TO_SDF_H
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include "CubeHandler.h"
#include "Geometry/Structure/KDTree.h"
#include "Geometry/Structure/HalfEdge.h"
#include "Geometry/Structure/PointCloud.h"
// Naive way to generate sdf from triangle mesh.
// for each voxel, use kdtree to find the closest vertex.
// then, we compute the distances from the voxel to the vertex-connected faces, the final distance will be the minumum.
// The sign of distance will be determined by the face normal.
namespace dragon
{
namespace reconstruction
{
    void Mesh2SDF(const geometry::TriangleMesh &mesh, CubeHandler &cube_handler, float voxel_resolution = 0.01);
    void NormalPCD2Indicator(const geometry::PointCloud &pcd, CubeHandler &cube_handler, float voxel_resolution = 0.01);
    void CenterLine2SDF(const geometry::Point3List &centers, const std::vector<double> &radius, CubeHandler &cube_handler, float voxel_resolution = 0.01);
}
}
#endif