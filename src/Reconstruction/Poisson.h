#ifndef DRAGON_POISSON_H
#define DRAGON_POISSON_H
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include "Reconstruction/CubeHandler.h"
// 1. build the octree
// 2. compute basis function, i.e. each node function
// 3. use basis function to compute the normal vector of each node
// 4. compute divergence, construct the L matrix
namespace dragon
{
namespace reconstruction
{
    std::shared_ptr<geometry::TriangleMesh> Poisson(const geometry::PointCloud &pcd, int max_depth);
    // start: [-0.5, 0.5], and the n-th filter range is [-0.5 * n, 0.5 * n]
    double ThirdBoxFilter(double x);
    double  ThirdBoxFilter3D(const geometry::Point3 &p);
}
}
#endif