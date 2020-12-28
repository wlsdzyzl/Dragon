#include "Geometry/Structure/HalfEdge.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"

namespace dragon
{
namespace geometry
{
namespace mesh
{
    std::shared_ptr<TriangleMesh> QuadricDecimation(const TriangleMesh &mesh, size_t target_num, double error_threshold);
}
}
}