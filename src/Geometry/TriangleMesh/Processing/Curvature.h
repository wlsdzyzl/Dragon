// compute mean curvature and the gauss curvature
#include "../HalfEdge.h"

namespace dragon
{
namespace geometry
{
    // compute mean curvature
    void ComputeMeanCurvature(const HalfEdge &he, Point3List &mean_curvature_vectors);
    // compute gauss curvature
    void ComputeGaussCurvature(const HalfEdge &he, std::vector<double> &gauss_curvature);

}
}