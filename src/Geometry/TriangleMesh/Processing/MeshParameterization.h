#ifndef DRAGON_MESH_PARA_H
#define DRAGON_MESH_PARA_H
#include "Geometry/Parameterization.h"
#include "Geometry/Structure/HalfEdge.h"

namespace dragon
{
namespace geometry
{
namespace mesh
{
    void GetOrderedVertices(HalfEdge &he, std::vector<int> &ordered_vid);
    std::shared_ptr<TriangleMesh > MeshParameterizationCircle(const TriangleMesh &mesh, int para_type = 0, double radius = 2);
    std::shared_ptr<TriangleMesh > MeshParameterizationSquare(const TriangleMesh &mesh, int para_type = 0, double len = 2);
}
}
}
#endif