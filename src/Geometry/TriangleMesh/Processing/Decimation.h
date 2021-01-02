#ifndef DRAGON_DECIMATION_H
#define DRAGON_DECIMATION_H
#include "Geometry/Structure/HalfEdge.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include <queue>
#include <unordered_map>
namespace dragon
{
namespace geometry
{
namespace mesh
{
    struct HEEdgeWithNewPos
    {
        HEEdge *ptr = nullptr;
        Point3 new_pos;
        HEEdgeWithNewPos() = default;
        HEEdgeWithNewPos(HEEdge *p, Point3 np):ptr(p), new_pos(np){}
    };
    struct GreaterHEEdge
    {
        inline bool operator()(const HEEdgeWithNewPos &a, const HEEdgeWithNewPos &b)
        {
            return a.ptr-> weight > b.ptr->weight;
            // if(a.ptr->weight !=  b.ptr->weight) 
            // else 
            // return (a.ptr->ori_vertex->coor - a.ptr->des_vertex->coor).norm() > (b.ptr->ori_vertex->coor - b.ptr->des_vertex->coor).norm(); 
        }
    };
    extern Vector4 zero_one;
    class Decimator
    {
        public:
        HalfEdge he;
        double error_threshold;
        void LoadTriangleMesh(const TriangleMesh &mesh);
        std::shared_ptr<TriangleMesh> Collapse(int target_num);
        std::shared_ptr<TriangleMesh> Cluster(double grid_len);
        bool Flipped(HEVertex *v, const Point3 &np, HEFace *f1, HEFace *f2);
        bool Flipped(HEEdgeWithNewPos &ep);
        bool Flipped(HEVertex *v1, HEVertex *v2);
        void DeleteFace(HEFace *f);
        void DeleteEdge(HEEdge *e);
        void QuadricConfig();
        void ClusteringConfig();
        void DirtyFace(HEFace *f);
        void DeleteDegeneratedFace();
        // void UpdateHalfEdge();
        int max_iteration = 50;
        inline bool IsValid(HEEdge * e){return e && e->id != -1;}
        inline bool IsValid(HEFace * f){return f && f->id != -1;}
        inline bool IsValid(HEVertex *v){return v && he.vertices[v->id] == v;}

        
        protected:
        std::tuple<Point3, double> ComputeEdgeError(int eid);
        Mat4List q_mat;
        // edge_error will be compacted in edge weight
        // ScalarList edge_error;
        // vertex
        int current_triangle_num;
        std::vector<bool> dirty;
        std::vector<int> updated;
        std::vector<std::vector<HEFace *>> vertex_to_faces;
        std::vector<size_t> clustered_vertex_num;
        // new position of an contracted edge.
        Point3List new_pos;
        // if a vertex's id is -1, it's deleted.
        // std::vector<bool> deleted;
        Vec4List planes;
        // based on the error.
        std::priority_queue<HEEdgeWithNewPos , std::vector<HEEdgeWithNewPos >, GreaterHEEdge> edge_queue;
        std::unordered_map<Point3i, HEVertex *, geometry::VoxelGridHasher > grid_map;
    };
    std::shared_ptr<TriangleMesh> QuadricDecimation(const TriangleMesh &mesh, size_t target_num);
    std::shared_ptr<TriangleMesh> ClusteringDecimation(const TriangleMesh &mesh, double voxel_len);
}
}
}
#endif