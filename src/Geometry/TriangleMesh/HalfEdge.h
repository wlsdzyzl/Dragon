#ifndef HALF_EDGE_H
#define HALF_EDGE_H
#include "Geometry/TriangleMesh/TriangleMesh.h"
namespace dragon
{
namespace geometry
{
namespace mesh
{
    struct HEEdge;
    struct HEFace;
    struct HEVertex
    {
        HEVertex()=default;
        HEVertex(const geometry::Point3 & p):coor(p){}
        //vertex of HalfEdge
        geometry::Point3 coor;
        geometry::Point3 color;
        //one edge whose start point is this vertex. 
        size_t id;
        HEEdge * inc_edge = nullptr; 
    };
    struct HEEdge
    {
        //edge of HalfEdge
        HEEdge() = default;
        HEEdge(HEVertex * v1, HEVertex * v2):ori_vertex(v1), des_vertex(v2){}
        HEEdge *twin_edge = nullptr;
        HEFace *parent_face = nullptr;
        HEVertex *ori_vertex = nullptr;
        HEVertex *des_vertex = nullptr;
        HEEdge *pre_edge = nullptr;
        HEEdge *next_edge = nullptr;
        double weight = -1.0;
        // size_t id;
    };
    struct HEFace
    {
        //face of HalfEdge
        HEFace() = default;
        HEFace(HEEdge * e): inc_edge(e){}
        HEEdge * inc_edge = nullptr;
        size_t id;
    };

    class HalfEdge
    {
        public:
        std::vector<HEVertex > vertices;
        std::vector<HEEdge > edges;
        std::vector<HEFace > faces;
        bool has_colors = false;
        std::vector<bool> is_border;
        ~HalfEdge()
        {
            Reset();
        }
        void Reset()
        {
            vertices.clear();
            edges.clear();
            faces.clear();
        }
        void FromTriangleMesh(const TriangleMesh &mesh);
        void ToTriangleMesh(TriangleMesh &mesh);
        void CheckBorder();
        void ResetWeight()
        {
            for(size_t i = 0; i != edges.size(); ++i)
                edges[i].weight = -1.0;
        }
    };
}
}
}
#endif