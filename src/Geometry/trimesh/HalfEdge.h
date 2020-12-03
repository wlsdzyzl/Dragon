#ifndef HALF_EDGE_H
#define HALF_EDGE_H
#include "Geometry/BasicGeometry.h"
namespace dragon
{
namespace trimesh
{
    struct HEEdge;
    struct HEFace;
    struct HEVertex
    {
        //vertex of HalfEdge
        geometry::Point3 coor; 
        HEEdge * inc_edge; 
    };
    struct HEEdge
    {
        //edge of HalfEdge
        HEEdge *twin_edge;
        HEFace *left_face;
        HEVertex *ori_vertex;
        HEVertex *des_vertex;
        HEEdge *pre_edge;
        HEEdge *next_edge;
    };
    struct HEFace
    {
        //face of HalfEdge
        HEEdge * inc_edge;
    };

    class HalfEdge
    {
        std::vector<HEVertex> vertices;
        std::vector<HEEdge> edges;
        std::vector<HEFace> faces;
    };
}
}
#endif