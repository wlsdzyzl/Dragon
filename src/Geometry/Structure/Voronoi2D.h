#ifndef DRAGON_VORONOI_H
#define DRAGON_VORONOI_H
#include "HalfEdge.h"
#include "Geometry/TriangleMesh/TriangleMesh.h"
#include "PointCloud.h"
#include <queue>
#include <unordered_map>
namespace dragon
{
namespace geometry
{
    struct ArcNode;
    struct CircleEvent
    {
        CircleEvent()=default;
        CircleEvent(double x_, Point2 o_, ArcNode * arc_):x(x_), o(o_), arc_ptr(arc_){}
        double x;
        Point2 o;
        ArcNode * arc_ptr;  
    };
    struct SiteEvent
    {
        SiteEvent(double x_, size_t id_):x(x_),id(id_){}
        double x;
        size_t id;
    };
    struct ArcNode
    {
        Point2 p;
        size_t id = -1;
        ArcNode() = default;
        ArcNode(const Point2 &p_, size_t id = -1):p(p_), id(id){}
        ArcNode *pre_ptr = nullptr;
        ArcNode *next_ptr = nullptr;
        CircleEvent * circle_events;
    };
    struct GreaterCircleEvent
    {
        inline bool operator()(const CircleEvent *a, const CircleEvent *b)
        {
            return a->x > b->x;
        }
    };
    struct GreaterSiteEvent
    {
        inline bool operator()(const SiteEvent *a, const SiteEvent *b)
        {
            return a->x > b->x;
        }
    };
    class Voronoi2D
    {
        public:
        Voronoi2D() = default;
        // 3d point
        // void FromPointCloud(const PointCloud &pcd);
        void FromPoints(const Point2List &points) {site_points = points;}
        void ToDualTriangleMesh(mesh::TriangleMesh &mesh) const;
        void Relaxation(int max_iteration = 100);
        void GenerateDiagram();
        // clockwise halfedge
        HalfEdge he;
        Point2List site_points;
        BoundingBox bb;
        double margin = 20;
        protected:
        void ProcessTopSiteEvent();
        void ProcessTopCircleEvent();
        void CheckCircleEvent(ArcNode *arc, double swp);
        void PrintArc() const;
        void BBTruncation();
        std::unordered_map<Point2i, int, PixelGridHasher> decisive_point_to_edge;
        Point2iList edge_to_decisive_point; 
        // site events queue
        std::priority_queue<SiteEvent *, std::vector<SiteEvent *>, GreaterSiteEvent> site_events;
        // circle events queue
        std::priority_queue<CircleEvent *, std::vector<CircleEvent *>, GreaterCircleEvent> circle_events;
        ArcNode * beachline = nullptr;

    };
}
}
#endif