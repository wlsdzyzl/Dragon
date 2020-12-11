#ifndef DRAGON_MESH_PARA_H
#define DRAGON_MESH_PARA_H
#include "Geometry/Parameterization.h"
#include "../HalfEdge.h"

namespace dragon
{
namespace geometry
{
namespace mesh
{
    void GetOrderedVertices(HalfEdge &he, std::vector<int> &ordered_vid);
    std::shared_ptr<TriangleMesh > MeshParameterization(const TriangleMesh &mesh, int para_type = 0, double radius = 2);
}
}
}
#endif