#ifndef DRAGON_SMOOTHING_H
#define DRAGON_SMOOTHING_H
#include "Geometry/Structure/HalfEdge.h"

namespace dragon
{
namespace geometry
{
namespace mesh
{
    std::shared_ptr<TriangleMesh> NaiveLaplacianSmooting(const TriangleMesh &mesh, double lambda = 0.1, int max_iteration = 100);
    std::shared_ptr<TriangleMesh> LocalLaplacianSmooting(const TriangleMesh &mesh, double lambda = 0.1, int max_iteration = 500);
    //internal implementation
    void GlobalLaplacianSmooting(HalfEdge &he, double lambda);
    std::shared_ptr<TriangleMesh> GlobalLaplacianSmooting(const TriangleMesh &mesh, double lambda = 0.9);
    
}
}
}
#endif