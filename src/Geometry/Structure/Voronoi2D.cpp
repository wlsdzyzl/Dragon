#include "Voronoi2D.h"

namespace dragon
{
namespace geometry
{
    // helpful function
    Point2 Intersection(const Point2 &p0, const Point2 &p1, double sweep_line)
    {
        Point2 res, p = p0;
        double squared_sweep = sweep_line * sweep_line;
        if(p0(0) == p1(0))
        {
            res(1) = (p0(1) + p1(1)) * 0.5;
        }
        else if(p0(0) == sweep_line)
        {
            // the parabola equation become y = p0(1)
            res(1) = p0(1);
            p = p1;
        }
        else if(p1(0) == sweep_line)
        {
            res(1) = p1(1);
        }
        else
        {
            // quadraic formula
            double z0 = 0.5 / (p0(0) - sweep_line);
            double z1 = 0.5 / (p1(0) - sweep_line);
            double a = z0 - z1;
            double b = 2 * p1(1) * z1 - 2 * p0(1) * z0;
            double c = (p0.squaredNorm() - squared_sweep) * z0 - (p1.squaredNorm() - squared_sweep) * z1;
            res(1) = (-b + sqrt(b *b - 4 * a* c))/(2 * a);
            // if (p0(1) = p1(1) && p0(0) < p1(0)), then this point is upon the line (p0-p1) 
            // std::cout<<"b+- 4ac: "<<(-b - sqrt(b *b - 4 * a* c))/(2 * a)<<" "<<(-b + sqrt(b *b - 4 * a* c))/(2 * a)<<std::endl;
        }
        // p(0) == sweep_line should never happen
        res(0) = (res(1)*res(1) - 2 * p(1) * res(1) + p.squaredNorm() - squared_sweep ) / (2 * (p(0) - sweep_line));
        return res;
    }
    bool Intersect(const Point2 &np, const ArcNode &arc, Point2 &res)
    {
        // the arc torch the new point, could happen when two points have the same x coordinate.
        if(np(0) == arc.p(0)) return false;
        Point2 pre_point, next_point;
        if(arc.pre_ptr != nullptr)
        {
            pre_point = Intersection(arc.pre_ptr->p, arc.p, np(0));
        }
        if(arc.next_ptr != nullptr)
        {   
            // next_point = Intersection(arc.p, arc.next_ptr->p, np(0));
            // std::cout<<" next: "<<next_point.transpose()<<std::endl;
            next_point = Intersection(arc.p, arc.next_ptr->p, np(0));
            // std::cout<<" next: "<<next_point.transpose()<<std::endl;
        }
        if((arc.pre_ptr == nullptr || pre_point(1) >= np(1)) && (arc.next_ptr == nullptr || next_point(1) <= np(1)) )
        {
            // std::cout<<"pre: "<<pre_point.transpose() <<" next: "<<next_point.transpose()<<std::endl;
            res = Intersection(arc.p, np, np(0));
            return true;
        }
        return false;
    }
    void Voronoi2D::CheckCircleEvent(ArcNode *arc, double swp)
    {
        int current_id = arc->id;
        int pre_id = -1, next_id = -1;
        // PrintArc();
        if(arc->pre_ptr) pre_id = arc->pre_ptr->id;
        if(arc->next_ptr) next_id = arc->next_ptr->id;
        // std::cout<<pre_id<<" "<<current_id << " "<<next_id<<std::endl;
        if(pre_id == -1 || next_id == -1 || pre_id == next_id)
        return;

        Point2 o;
        double r, d;
        std::tie(o, r, d) = CircumCenter(site_points[current_id], site_points[pre_id], site_points[next_id]);
        // std::cout<<"is clockwise? "<<d<<std::endl;
        if(d >-EPS) return;
        if(o(0) + r >=swp)
        {
            CircleEvent *ce = new CircleEvent(o(0) + r, o, arc);
            circle_events.push(ce);
            std::cout<<GREEN<<"Push Circle Event:"<<o(0) + r<<RESET<<std::endl;
        }
    }
    void Voronoi2D::ProcessTopSiteEvent()
    {
        if(!site_events.empty())
        {
            auto top = site_events.top();
            site_events.pop();
            std::cout<<GREEN<<"Process Site Event:"<<top->x<<RESET<<std::endl;
            while(!circle_events.empty())
            {
                auto tmp_top = circle_events.top();
                circle_events.pop();
                delete tmp_top;
            }
            auto &np = site_points[top->id];
            // binary tree could be faster to localte the arc.
            if(beachline == nullptr)
            {
                beachline = new ArcNode(np, top->id);
            }
            else
            {
                auto tmp_arc = beachline;
                while(tmp_arc != nullptr)
                {
                    Point2 res;
                    //if there is a breakpoint
                    std::cout<<"check if arc "<<tmp_arc->id<<" ..."<<std::endl;
                    if(Intersect(np, *tmp_arc, res))
                    {
                        std::cout<<GREEN<<"Add new arc: "<<np.transpose()<<" on arc " <<tmp_arc->id<<RESET<<std::endl;
                        ArcNode * narc = new ArcNode(np, top->id);
                        ArcNode * splited = new ArcNode(*tmp_arc);
                        
                        narc->pre_ptr = tmp_arc;
                        narc->next_ptr = splited;
                        splited->pre_ptr = narc;
                        if(splited->next_ptr != nullptr)
                        splited->next_ptr->pre_ptr = splited;
                        tmp_arc->next_ptr = narc;
                        
                        // PrintArc();
                        CheckCircleEvent(narc, np(0));
                        CheckCircleEvent(tmp_arc, np(0));
                        CheckCircleEvent(splited, np(0));

                        // Add edge into halfedge
                        // edge.des = nullptr
                        // this edge may not have an end point. it defined by these two points
                        decisive_point_to_edge[Point2i(top->id, tmp_arc->id)] = he.edges.size();
                        he.edges.push_back(HEEdge());
                        HEEdge &edge0 = he.edges.back();
                        edge0.id = he.edges.size()-1;
                        edge_to_decisive_point.push_back(Point2i(top->id, tmp_arc->id));
                        decisive_point_to_edge[Point2i(tmp_arc->id, top->id)] = he.edges.size();
                        he.edges.push_back(HEEdge());
                        HEEdge &edge1 = he.edges.back();
                        edge1.id = he.edges.size()-1;
                        edge_to_decisive_point.push_back(Point2i(tmp_arc->id, top->id));
                        // std::cout<<&edge1<<" "<<&he.edges.back()<<std::endl;
                        edge1.twin_edge = &edge0;
                        edge0.twin_edge = &edge1;
                        
                        break;
                    }
                    else 
                    tmp_arc = tmp_arc->next_ptr;
                }
                // check if there is a circle event.
            }
            delete top;
        }
        else return;
    }
    void Voronoi2D::ProcessTopCircleEvent()
    {
        if(!circle_events.empty())
        {
            
            auto top = circle_events.top();
            circle_events.pop();
            //this arc is going to disappear
            ArcNode *arc = top->arc_ptr;            
            // PrintArc();
            std::cout<<GREEN<<"Process Circle Event:"<<top->x<<RESET<<std::endl;
            int edge_id_0 = -1, edge_id_1= -1; 
            if(arc->next_ptr && arc->pre_ptr)
            {
                //connect the edge to the vertex. 
                edge_id_0 = decisive_point_to_edge[Point2i(arc->pre_ptr->id, arc->id)];
                edge_id_1 = decisive_point_to_edge[Point2i(arc->id, arc->next_ptr->id)];
                if(he.edges[edge_id_0].ori_vertex != nullptr) edge_id_0 = decisive_point_to_edge[Point2i(arc->id, arc->pre_ptr->id)];
                if(he.edges[edge_id_1].ori_vertex != nullptr) edge_id_1 = decisive_point_to_edge[Point2i(arc->next_ptr->id, arc->id)];
                he.vertices.push_back(HEVertex());
                HEVertex *hev_ptr = &he.vertices.back();
                hev_ptr->id = he.vertices.size() - 1;
                hev_ptr->coor= top->o;
                // std::cout<<he.edges[edge_id_0].ori_vertex<<" "<<he.edges[edge_id_1].ori_vertex<<" "<<he.edges[edge_id_0].twin_edge->des_vertex<<" "<<he.edges[edge_id_1].twin_edge->des_vertex  <<std::endl;
                // std::cout<<he.edges[edge_id_0].id<<" "<<he.edges[edge_id_0].twin_edge->id<<" "<<he.edges[edge_id_1].id<<" "<<he.edges[edge_id_1].twin_edge->id  <<std::endl;
                he.edges[edge_id_0].ori_vertex = hev_ptr;
                he.edges[edge_id_0].twin_edge->des_vertex = hev_ptr;
                he.edges[edge_id_1].ori_vertex = hev_ptr;
                he.edges[edge_id_1].twin_edge->des_vertex = hev_ptr;
                if(hev_ptr->inc_edge == nullptr)
                {
                    hev_ptr->inc_edge = &he.edges[edge_id_0];
                }
                arc->next_ptr->pre_ptr = arc->pre_ptr;
                arc->pre_ptr->next_ptr = arc->next_ptr;


                // add new edge
                decisive_point_to_edge[Point2i(arc->pre_ptr->id, arc->next_ptr->id)] = he.edges.size();
                he.edges.push_back(HEEdge());
                HEEdge &nedge0 = he.edges.back();
                nedge0.id = he.edges.size()-1;
                edge_to_decisive_point.push_back(Point2i(arc->pre_ptr->id, arc->next_ptr->id));

                decisive_point_to_edge[Point2i(arc->next_ptr->id, arc->pre_ptr->id)] = he.edges.size();
                he.edges.push_back(HEEdge());
                HEEdge &nedge1 = he.edges.back();
                nedge1.id = he.edges.size()-1;
                edge_to_decisive_point.push_back(Point2i(arc->next_ptr->id, arc->pre_ptr->id));

                nedge0.ori_vertex = hev_ptr;
                nedge1.des_vertex = hev_ptr;
                nedge0.twin_edge = &nedge1;
                nedge1.twin_edge = &nedge0;
                
                // connect edges
                nedge1.next_edge = &he.edges[edge_id_1];
                he.edges[edge_id_1].pre_edge = &nedge1;

                nedge0.pre_edge = he.edges[edge_id_0].twin_edge;
                he.edges[edge_id_0].twin_edge->next_edge = &nedge0;

                he.edges[edge_id_1].twin_edge->next_edge = &he.edges[edge_id_0];
                he.edges[edge_id_0].pre_edge = he.edges[edge_id_1].twin_edge;

                // face
                nedge1.parent_face = &he.faces[arc->next_ptr->id];
                he.edges[edge_id_1].parent_face= &he.faces[arc->next_ptr->id];
                if(he.faces[arc->next_ptr->id].inc_edge == nullptr) he.faces[arc->next_ptr->id].inc_edge = &nedge1;

                nedge0.parent_face = &he.faces[arc->pre_ptr->id];
                he.edges[edge_id_0].twin_edge->parent_face = &he.faces[arc->pre_ptr->id];
                if(he.faces[arc->pre_ptr->id].inc_edge == nullptr) he.faces[arc->pre_ptr->id].inc_edge = &nedge0;

                he.edges[edge_id_1].twin_edge->parent_face = &he.faces[arc->id];
                he.edges[edge_id_0].parent_face = &he.faces[arc->id];                
                if(he.faces[arc->id].inc_edge == nullptr) he.faces[arc->id].inc_edge = &he.edges[edge_id_0];
            }
            else
            {
                std::cout<<RED<<"[ERROR]::[CircleEvent]::Something wrong."<<RESET<<std::endl;
                exit(0);
            }
            CheckCircleEvent(arc->pre_ptr, top->x);
            CheckCircleEvent(arc->next_ptr, top->x);
            // PrintArc();
            delete arc;
        }
        else return;
    }
    void Voronoi2D::Relaxation(int max_iteration)
    {
        //iteration
        int iteration = 0;
        auto &vertices = he.vertices;
        while(iteration < max_iteration)
        {
            for(size_t i = 0; i != vertices.size(); ++i)
            {
                // geometry::VectorX p = geometry::Point3;
            }
        }
    }
    void Voronoi2D::PrintArc() const
    {
        auto tmp_arc = beachline;
        std::cout <<"arc: ";
        while(tmp_arc != nullptr)
        {
            std::cout<<tmp_arc->id<<" ";
            tmp_arc = tmp_arc->next_ptr;
            // tmp_arc = tmp_arc->pre_ptr;
        }
        tmp_arc = beachline;
        while(tmp_arc->next_ptr != nullptr)
        {
            tmp_arc = tmp_arc->next_ptr;
        }
        std::cout<<" | ";
        while(tmp_arc != nullptr)
        {
            std::cout<<tmp_arc->id<<" ";
            // tmp_arc = tmp_arc->next_ptr;
            tmp_arc = tmp_arc->pre_ptr;
        }
        std::cout << std::endl;
    }
    void Voronoi2D::BBTruncation()
    {
        // this file is to use boundingbox to truncate the edges, which has a nullptr vertex
        // auto &faces = he.faces;
        auto &edges = he.edges;
        auto &vertices = he.vertices;
        for(size_t i = 0; i != site_points.size(); ++i)
        {
            bb.AddPoint(Point3(site_points[i](0), site_points[i](1), 0.0));
        }
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            bb.AddPoint(Point3(vertices[i].coor(0), vertices[i].coor(1), 0.0));
        }
        bb.x_max += margin;
        bb.x_min -= margin;
        bb.y_max += margin;
        bb.y_min -= margin;
        Point2 left_top(bb.x_min, bb.y_max), left_bottom(bb.x_min, bb.y_min);
        Point2 right_bottom(bb.x_max, bb.y_min), right_top(bb.x_max, bb.y_max);
        LineSegment left(left_top, left_bottom), right(right_top, right_bottom);
        LineSegment up(left_top, right_top), down(left_bottom, right_bottom);
        for(size_t i = 0; i != edges.size(); ++i)
        {
            if(edges[i].ori_vertex && edges[i].des_vertex) continue;
            if(!edges[i].ori_vertex && !edges[i].des_vertex)
            {
                std::cout<<RED<<"[ERROR]::[BBTruncation]::Something wrong."<<RESET<<std::endl;
                exit(0);
            }
            int face_id = edges[i].parent_face->id;
            if(face_id != -1)
            {
                auto site_point =  site_points[face_id];
                Line vb = VerticalBisector(site_points [ edge_to_decisive_point[i](0)], site_points[edge_to_decisive_point[i](1)]);
                // compute the intersection of BB

                Point2List inter_points;
                if(IsIntersecting(vb, left))
                {
                    inter_points.push_back(LineSegIntersect(vb, left));
                }
                if(IsIntersecting(vb, right))
                {
                    inter_points.push_back(LineSegIntersect(vb, right));
                }
                if(IsIntersecting(vb, up))
                {
                    inter_points.push_back(LineSegIntersect(vb, up));
                }
                if(IsIntersecting(vb, down))
                {
                    inter_points.push_back(LineSegIntersect(vb, down));
                }
                if(edges[i].ori_vertex)
                {

                    auto ori_point = edges[i].ori_vertex->coor;
                    // std::cout<<site_points [ edge_to_decisive_point[i](0)].transpose()<<std::endl;
                    // std::cout<<site_points [ edge_to_decisive_point[i](1)].transpose()<<std::endl;
                    // std::cout<<ori_point.transpose()<<std::endl;
                    // std::cout<<vb.n.transpose()<<std::endl;
                    // std::cout<<"---"<<std::endl;
                    for(size_t j = 0; j != inter_points.size(); ++j)
                    {
                        if(CheckPointToLine(ori_point, inter_points[j], site_point) > 0)
                        {
                            vertices.push_back(HEVertex(inter_points[j]));
                            vertices.back().id = vertices.size() - 1;
                            edges[i].des_vertex = &vertices.back() ;
                            edges[i].twin_edge->ori_vertex = &vertices.back();
                            break;
                        }
                    }
                }
                else
                {
                    auto des_point = edges[i].des_vertex->coor;
                    for(size_t j = 0; j != inter_points.size(); ++j)
                    {
                        if(CheckPointToLine(inter_points[j], des_point, site_point) > 0)
                        {
                            vertices.push_back(HEVertex(inter_points[j]));
                            vertices.back().id = vertices.size() - 1;
                            edges[i].ori_vertex = &vertices.back();
                            edges[i].twin_edge->des_vertex = &vertices.back();
                            break;
                        }
                    }
                }
            }
        }
    }
    void Voronoi2D::GenerateDiagram()
    {
        std::priority_queue<SiteEvent *, std::vector<SiteEvent *>, GreaterSiteEvent>().swap(site_events);
        std::priority_queue<CircleEvent *, std::vector<CircleEvent *>, GreaterCircleEvent>().swap(circle_events);
        for(size_t i = 0; i != site_points.size(); ++i)
        {
            //add site events
            SiteEvent *tmp_se = new SiteEvent(site_points[i](0), i);
            site_events.push(tmp_se);
        }
        //max number of voronoi edges: 3n-6
        //max number of voronoi vertex: 2n-5
        //max number of voronoi face: n
        he.vertices.reserve(2 * site_points.size());
        he.edges.reserve(3 * site_points.size() * 2);
        he.faces.resize(site_points.size());
        for(size_t i = 0; i != site_points.size(); ++i)
            he.faces[i].id = i;
        while(!site_events.empty())
        {
            // reconsider when 4 points on the same circle.
            if(!circle_events.empty() && circle_events.top()->x <= site_events.top()->x)
                ProcessTopCircleEvent();
            else
                ProcessTopSiteEvent();
        }

        while(!circle_events.empty())
        {
            ProcessTopCircleEvent();
        }
        BBTruncation();
        std::cout<<"voronoi: "<<he.faces.size()<<" faces, "<<he.edges.size()<<" edges, "<<he.vertices.size()<<" vertices."<<std::endl;
    }
}
}