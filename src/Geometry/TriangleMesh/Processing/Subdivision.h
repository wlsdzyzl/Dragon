#ifndef DRAGON_SUBDIVISION_H
#define DRAGON_SUBDIVISION_H
#include "Geometry/Structure/HalfEdge.h"

namespace dragon
{
namespace geometry
{
namespace mesh
{
    std::shared_ptr<TriangleMesh> LoopSubdivision(const TriangleMesh &mesh, int iteration = 1);
    
}
}
}
#endif