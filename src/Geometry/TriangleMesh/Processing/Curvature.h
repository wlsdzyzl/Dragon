// compute mean curvature and the gauss curvature
#include "../HalfEdge.h"

namespace dragon
{
namespace geometry
{
    // compute mean curvature
    void ComputeMeanCurvature(HalfEdge &he, Point3List &mean_curvature_vectors);
    // compute gauss curvature
    void ComputeGaussCurvature(HalfEdge &he, std::vector<double> &gauss_curvature);
    void ComputeLocalWeights(HalfEdge &he);
}
}