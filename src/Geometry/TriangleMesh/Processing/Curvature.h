#ifndef DRAGON_GURVATURE_H
#define DRAGON_GURVATURE_H
// compute mean curvature and the gauss curvature
#include "Geometry/Structure/HalfEdge.h"

namespace dragon
{
namespace geometry
{
namespace mesh
{
    // compute mean curvature
    //void ComputeMeanCurvature(HalfEdge &he, Point3List &mean_curvature_vectors);
    void ComputeMeanCurvature(HalfEdge &he, geometry::ScalarList &mean_curvatures);
    // compute gauss curvature
    void ComputeGaussCurvature(HalfEdge &he, geometry::ScalarList &gauss_curvature);
    void ComputeCotanWeight(HalfEdge &he);
}
}
}
#endif