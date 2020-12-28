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
        bool is_valid = true;
    };
    struct SiteEvent
    {
        SiteEvent(double x_, double y_, size_t id_):x(x_), y(y_),id(id_){}
        double x;
        double y;
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
        CircleEvent * circle_event = nullptr;
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
            if(a->x !=  b->x) return a-> x > b->x;
            else return a->y < b->y; 
        }
    };
    class Voronoi2D
    {
        public:
        Voronoi2D() = default;
        // 3d point
        // void FromPointCloud(const PointCloud &pcd);
        void FromPoints(const Point2List &points) { Reset(); site_points = points;}
        void ToDualTriangleMesh(TriangleMesh &mesh) const;
        Point2List Relaxation();
        void GenerateDiagram();
        void Reset()
        {
            he.Reset();
            decisive_point_to_edge.clear();
            edge_to_decisive_point.clear();
            bb = BoundingBox();
            site_points.clear();
            while(beachline!= nullptr)
            {
                auto tmp_arc = beachline->next_ptr;
                delete beachline;
                beachline = tmp_arc;
            }
            beachline = nullptr;
        }
        void BBTruncation(const BoundingBox &bb);
        // clockwise halfedge
        HalfEdge he;
        Point2List site_points;
        BoundingBox bb;
        // double margin = 20;
        protected:
        void ProcessTopSiteEvent();
        void ProcessTopCircleEvent();
        void CheckCircleEvent(ArcNode *arc, double swp);
        void PrintArc() const;
        
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