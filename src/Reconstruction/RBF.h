#ifndef DRAGON_RBF_H
#define DRAGON_RBF_H
#include "Geometry/BasicGeometry.h"
#include "Geometry/Structure/PointCloud.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include "MarchingCube.h"
#include "CubeHandler.h"
namespace dragon
{
namespace reconstruction
{
    // we choose basis function as phi(r) = r^3
    void GenerateSamplePoints(const geometry::PointCloud &pcd, geometry::Vec4List &samp, double e = 1);
    std::shared_ptr<geometry::TriangleMesh> RBF(const geometry::PointCloud &pcd, CubeHandler &cube_handler, double e = 1);
}
}
#endif