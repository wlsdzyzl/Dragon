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
    std::shared_ptr<geometry::TriangleMesh> RBF(const geometry::PointCloud &pcd, CubeHandler &cube_handler);
}
}
#endif