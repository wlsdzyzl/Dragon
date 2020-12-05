#include "../HalfEdge.h"

namespace dragon
{
namespace geometry
{
    void LaplacianSmooting(const HalfEdge &he, Point3List &next_position);
    std::shared_ptr<TriangleMesh> LaplacianSmooting(const TriangleMesh &mesh, int max_iteration = 100);
}
}