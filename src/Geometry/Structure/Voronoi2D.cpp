#include "Voronoi2D.h"
#include <iomanip>
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
    void Voronoi2D::ToDualTriangleMesh(TriangleMesh &mesh) const
    {
        // to dual triangle mesh, which is also known as delaunay triangulation
        if(site_points.size()==0) return;
        auto &vertices = he.vertices;
        mesh.Reset();
        for(size_t i = 0; i != site_points.size(); ++i)
        {
            mesh.points.push_back(Point3(site_points[i](0), site_points[i](1), 0));
        }
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            if(vertices[i]->id == -1) 
            {
                continue;
            }
            if(vertices[i]->inc_edge)
            {
                auto start_edge = vertices[i]->inc_edge;
                auto tmp_edge = start_edge;
                std::vector<int> fid;
                while(true)
                {
                    fid.push_back(tmp_edge->parent_face->id);
                    tmp_edge = tmp_edge->twin_edge->next_edge;
                    if(tmp_edge == start_edge)
                    break;
                    if(tmp_edge == nullptr)
                    {
                        std::cout<<YELLOW<<"[WARNING]::[ToDualTriangleMesh]::Null edge shouldn't exist."<<RESET<<std::endl;
                    }
                }
                if(fid.size() == 3)
                {
                    mesh.triangles.push_back(geometry::Point3ui(fid[0], fid[1], fid[2]));
                }
                else
                {
                    std::cout<<YELLOW<<"[WARNING]::[ToDualTriangleMesh]::One vertex has more than 6 edges."<<RESET<<std::endl;
                }
            }
        }
    }
    void Voronoi2D::CheckCircleEvent(ArcNode *arc, double swp)
    {
        int current_id = arc->id;
        int pre_id = -1, next_id = -1;
        // PrintArc();
        if(arc->pre_ptr) pre_id = arc->pre_ptr->id;
        if(arc->next_ptr) next_id = arc->next_ptr->id;
        // std::cout<<"check circle event: "<<pre_id<<" "<<current_id << " "<<next_id<<std::endl;
        if(pre_id == -1 || next_id == -1 || pre_id == next_id)
        return;

        Point2 o;
        double r, d;
        std::tie(o, r, d) = CircumCenter(site_points[current_id], site_points[pre_id], site_points[next_id]);
        // std::cout<<"is clockwise? "<<d<<std::endl;
        if(d >-EPS)
        {
            // std::cout<<d<< " "<<-EPS<<std::endl;
            return;
        }
        if(o(0) + r >=swp || std::fabs(o(0) + r - swp) < 0.01)
        {
            CircleEvent *ce = new CircleEvent(o(0) + r, o, arc);
            circle_events.push(ce);
            if(!arc->circle_event)
            {
                arc->circle_event = ce;
                std::cout<<GREEN<<"Push Circle Event: "<<o(0) + r<<RESET<<std::endl;
            }
            else
            {
                arc->circle_event->is_valid = false;
                arc->circle_event = ce;
                std::cout<<GREEN<<"Replace Circle Event: "<<o(0) + r<<RESET<<std::endl;
            }
        }
        else
        {
            std::cout<<RED<<std::setprecision(20) <<o(0)+r<<" "<<swp<<RESET<<std::endl;
        }
    }
    void Voronoi2D::ProcessTopSiteEvent()
    {
        if(!site_events.empty())
        {
            auto top = site_events.top();
            site_events.pop();
            std::cout<<GREEN<<"Process Site Event:"<<site_points[top->id].transpose()<<RESET<<std::endl;

            auto &np = site_points[top->id];
            // binary tree could be faster to localte the arc.
            if(beachline == nullptr)
            {
                beachline = new ArcNode(np, top->id);
            }
            else
            {
                auto tmp_arc = beachline;
                if(beachline->p(0) == np(0))
                {
                    while(true)
                    {
                        if(tmp_arc->next_ptr)
                        tmp_arc = tmp_arc->next_ptr;
                        else break;
                    }
                    //now tmp_arc is the end of beachline
                    ArcNode * narc = new ArcNode(np, top->id);
                    narc->pre_ptr = tmp_arc;
                    tmp_arc->next_ptr = narc;

                    decisive_point_to_edge[Point2i(top->id, tmp_arc->id)] = he.edges.size();
                    he.edges.push_back(new HEEdge());
                    HEEdge *edge0 = he.edges.back();
                    edge0->id = he.edges.size()-1;
                    edge_to_decisive_point.push_back(Point2i(top->id, tmp_arc->id));

                    decisive_point_to_edge[Point2i(tmp_arc->id, top->id)] = he.edges.size();
                    he.edges.push_back(new HEEdge());
                    HEEdge *edge1 = he.edges.back();
                    edge1->id = he.edges.size()-1;
                    edge_to_decisive_point.push_back(Point2i(tmp_arc->id, top->id));
                    // std::cout<<&edge1<<" "<<&he.edges.back()<<std::endl;
                    edge1->twin_edge = edge0;
                    edge0->twin_edge = edge1;
                }
                else 
                {
                    while(tmp_arc != nullptr)
                    {
                        Point2 res;
                        //if there is a breakpoint
                        // std::cout<<"check if arc "<<tmp_arc->id<<" ..."<<std::endl;

                        if(Intersect(np, *tmp_arc, res))
                        {
                            std::cout<<GREEN<<"Add new arc: "<<np.transpose()<<" on arc " <<tmp_arc->id<<RESET<<std::endl;
                            if(tmp_arc->circle_event != nullptr) 
                            {
                                tmp_arc->circle_event->is_valid = false;
                                tmp_arc->circle_event = nullptr;
                            }
                            ArcNode * narc = new ArcNode(np, top->id);
                            ArcNode * splited = new ArcNode(*tmp_arc);
                            
                            narc->pre_ptr = tmp_arc;
                            narc->next_ptr = splited;
                            splited->pre_ptr = narc;
                            if(splited->next_ptr != nullptr)
                            splited->next_ptr->pre_ptr = splited;
                            tmp_arc->next_ptr = narc;
                            


                            // Add edge into halfedge
                            // edge.des = nullptr
                            // this edge may not have an end point. it defined by these two points
                            decisive_point_to_edge[Point2i(top->id, tmp_arc->id)] = he.edges.size();
                            he.edges.push_back(new HEEdge());
                            HEEdge *edge0 = he.edges.back();
                            edge0->id = he.edges.size()-1;
                            edge_to_decisive_point.push_back(Point2i(top->id, tmp_arc->id));

                            decisive_point_to_edge[Point2i(tmp_arc->id, top->id)] = he.edges.size();
                            he.edges.push_back(new HEEdge());
                            HEEdge *edge1 = he.edges.back();
                            edge1->id = he.edges.size()-1;
                            edge_to_decisive_point.push_back(Point2i(tmp_arc->id, top->id));
                            // std::cout<<&edge1<<" "<<&he.edges.back()<<std::endl;
                            edge1->twin_edge = edge0;
                            edge0->twin_edge = edge1;
                            
                            // PrintArc();
                            CheckCircleEvent(narc, np(0));
                            CheckCircleEvent(tmp_arc, np(0));
                            CheckCircleEvent(splited, np(0));
                            break;
                        }
                        else 
                        tmp_arc = tmp_arc->next_ptr;
                    }
                    // check if there is a circle event.
                }
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
            if(!top->is_valid)
            {
                delete top;
                return;
            }
            //this arc is going to disappear
            ArcNode *arc = top->arc_ptr;            
            // PrintArc();
            
            int edge_id_0 = -1, edge_id_1= -1; 
            // clockwise
            if(arc->next_ptr && arc->pre_ptr)
            {
                std::cout<<GREEN<<"Process Circle Event: "<<top->o.transpose()<<RESET<<std::endl;
                //connect the edge to the vertex. 
                edge_id_0 = decisive_point_to_edge[Point2i(arc->pre_ptr->id, arc->id)];
                edge_id_1 = decisive_point_to_edge[Point2i(arc->id, arc->next_ptr->id)];
                if(he.edges[edge_id_0]->ori_vertex != nullptr) edge_id_0 = decisive_point_to_edge[Point2i(arc->id, arc->pre_ptr->id)];
                if(he.edges[edge_id_1]->ori_vertex != nullptr) edge_id_1 = decisive_point_to_edge[Point2i(arc->next_ptr->id, arc->id)];
                he.vertices.push_back(new HEVertex());
                HEVertex *hev_ptr = he.vertices.back();
                hev_ptr->id = he.vertices.size() - 1;
                hev_ptr->coor= top->o;
                // std::cout<<he.edges[edge_id_0].ori_vertex<<" "<<he.edges[edge_id_1].ori_vertex<<" "<<he.edges[edge_id_0].twin_edge->des_vertex<<" "<<he.edges[edge_id_1].twin_edge->des_vertex  <<std::endl;
                // std::cout<<he.edges[edge_id_0].id<<" "<<he.edges[edge_id_0].twin_edge->id<<" "<<he.edges[edge_id_1].id<<" "<<he.edges[edge_id_1].twin_edge->id  <<std::endl;
                he.edges[edge_id_0]->ori_vertex = hev_ptr;
                he.edges[edge_id_0]->twin_edge->des_vertex = hev_ptr;
                he.edges[edge_id_1]->ori_vertex = hev_ptr;
                he.edges[edge_id_1]->twin_edge->des_vertex = hev_ptr;
                if(hev_ptr->inc_edge == nullptr)
                {
                    hev_ptr->inc_edge = he.edges[edge_id_0];
                }
                arc->next_ptr->pre_ptr = arc->pre_ptr;
                arc->pre_ptr->next_ptr = arc->next_ptr;


                // add new edge
                decisive_point_to_edge[Point2i(arc->pre_ptr->id, arc->next_ptr->id)] = he.edges.size();
                he.edges.push_back(new HEEdge());
                HEEdge *nedge0 = he.edges.back();
                nedge0->id = he.edges.size()-1;
                edge_to_decisive_point.push_back(Point2i(arc->pre_ptr->id, arc->next_ptr->id));

                decisive_point_to_edge[Point2i(arc->next_ptr->id, arc->pre_ptr->id)] = he.edges.size();
                he.edges.push_back(new HEEdge());
                HEEdge *nedge1 = he.edges.back();
                nedge1->id = he.edges.size()-1;
                edge_to_decisive_point.push_back(Point2i(arc->next_ptr->id, arc->pre_ptr->id));

                nedge0->ori_vertex = hev_ptr;
                nedge1->des_vertex = hev_ptr;
                nedge0->twin_edge = nedge1;
                nedge1->twin_edge = nedge0;
                
                // connect edges
                // clockwise
                nedge0->pre_edge = he.edges[edge_id_1]->twin_edge;
                he.edges[edge_id_1]->twin_edge->next_edge = nedge0;

                nedge1->next_edge = he.edges[edge_id_0];
                he.edges[edge_id_0]->pre_edge = nedge1;

                he.edges[edge_id_0]->twin_edge->next_edge = he.edges[edge_id_1];
                he.edges[edge_id_1]->pre_edge = he.edges[edge_id_0]->twin_edge;

                // face
                nedge1->parent_face = he.faces[arc->pre_ptr->id];
                he.edges[edge_id_0]->parent_face= he.faces[arc->pre_ptr->id];
                if(he.faces[arc->pre_ptr->id]->inc_edge == nullptr) he.faces[arc->pre_ptr->id]->inc_edge = nedge1;

                nedge0->parent_face = he.faces[arc->next_ptr->id];
                he.edges[edge_id_1]->twin_edge->parent_face = he.faces[arc->next_ptr->id];
                if(he.faces[arc->next_ptr->id]->inc_edge == nullptr) he.faces[arc->next_ptr->id]->inc_edge = nedge0;

                he.edges[edge_id_0]->twin_edge->parent_face = he.faces[arc->id];
                he.edges[edge_id_1]->parent_face = he.faces[arc->id];                
                if(he.faces[arc->id]->inc_edge == nullptr) he.faces[arc->id]->inc_edge = he.edges[edge_id_1];                

                // // counter clock wise
                // nedge1->next_edge = he.edges[edge_id_1];
                // he.edges[edge_id_1]->pre_edge = nedge1;

                // nedge0->pre_edge = he.edges[edge_id_0]->twin_edge;
                // he.edges[edge_id_0]->twin_edge->next_edge = nedge0;

                // he.edges[edge_id_1]->twin_edge->next_edge = he.edges[edge_id_0];
                // he.edges[edge_id_0]->pre_edge = he.edges[edge_id_1]->twin_edge;

                // // face
                // nedge1->parent_face = he.faces[arc->next_ptr->id];
                // he.edges[edge_id_1]->parent_face= he.faces[arc->next_ptr->id];
                // if(he.faces[arc->next_ptr->id]->inc_edge == nullptr) he.faces[arc->next_ptr->id]->inc_edge = nedge1;

                // nedge0->parent_face = he.faces[arc->pre_ptr->id];
                // he.edges[edge_id_0]->twin_edge->parent_face = he.faces[arc->pre_ptr->id];
                // if(he.faces[arc->pre_ptr->id]->inc_edge == nullptr) he.faces[arc->pre_ptr->id]->inc_edge = nedge0;

                // he.edges[edge_id_1]->twin_edge->parent_face = he.faces[arc->id];
                // he.edges[edge_id_0]->parent_face = he.faces[arc->id];                
                // if(he.faces[arc->id]->inc_edge == nullptr) he.faces[arc->id]->inc_edge = he.edges[edge_id_0];
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
            delete top;
        }
        else return;
    }
    Point2List Voronoi2D::Relaxation()
    {
        // auto &vertices = he.vertices;
        // auto &edges = he.edges;
        auto &faces = he.faces;
        Point2List new_site_points;
        for(size_t i = 0; i != faces.size(); ++i)
        {
            auto start_edge = faces[i]->inc_edge;
            auto tmp_edge = start_edge;
            bool should_update = true;
            int vertex_num = 0;
            Point2 sum_point = Point2::Zero();
            
            while(true)
            {
                if(tmp_edge == nullptr)
                {
                    should_update = false;
                    break;
                }
                if(tmp_edge->ori_vertex)
                {
                    vertex_num ++;
                    sum_point += tmp_edge-> ori_vertex->coor;
                }
                else
                {
                    should_update = false;
                    std::cout<<YELLOW<<"[WARNING]::[Relaxation]::Edge with two nullptr vertex is abnormal."<<RESET<<std::endl;
                    break;
                }
                std::cout<<tmp_edge->id<<" ";
                tmp_edge = tmp_edge->next_edge;
                if(tmp_edge == start_edge) break;
            }
            std::cout<<std::endl;
            if(should_update) new_site_points.push_back(sum_point / vertex_num);
            else new_site_points.push_back(site_points[i]);
        } 
        return new_site_points;
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
    // void Voronoi2D::BBTruncation()
    // {
    //     // this file is to use boundingbox to truncate the edges, which has a nullptr vertex
    //     // auto &faces = he.faces;
    //     auto &edges = he.edges;
    //     auto &vertices = he.vertices;
    //     for(size_t i = 0; i != site_points.size(); ++i)
    //     {
    //         bb.AddPoint(Point3(site_points[i](0), site_points[i](1), 0.0));
    //     }
    //     for(size_t i = 0; i != vertices.size(); ++i)
    //     {
    //         bb.AddPoint(Point3(vertices[i]->coor(0), vertices[i]->coor(1), 0.0));
    //     }
    //     bb.x_max += margin;
    //     bb.x_min -= margin;
    //     bb.y_max += margin;
    //     bb.y_min -= margin;
    //     Point2 left_top(bb.x_min, bb.y_max), left_bottom(bb.x_min, bb.y_min);
    //     Point2 right_bottom(bb.x_max, bb.y_min), right_top(bb.x_max, bb.y_max);
    //     LineSegment left(left_top, left_bottom), right(right_top, right_bottom);
    //     LineSegment up(left_top, right_top), down(left_bottom, right_bottom);
    //     for(size_t i = 0; i != edges.size(); ++i)
    //     {
    //         if(edges[i]->ori_vertex && edges[i]->des_vertex) continue;
    //         if(!edges[i]->ori_vertex && !edges[i]->des_vertex)
    //         {
    //             std::cout<<YELLOW<<"[WARNING]::[BBTruncation]::Edge with two nullptr vertices is abnormal."<<RESET<<std::endl;
    //             continue;
    //             //exit(0);
    //         }
    //         int face_id = edges[i]->parent_face->id;
    //         if(face_id != -1)
    //         {
    //             auto site_point =  site_points[face_id];
    //             Line vb = VerticalBisector(site_points [ edge_to_decisive_point[i](0)], site_points[edge_to_decisive_point[i](1)]);
    //             // compute the intersection of BB
                
    //             Point2List inter_points;
    //             if(IsIntersecting(vb, left))
    //             {
    //                 inter_points.push_back(LineSegIntersect(vb, left));
    //             }
    //             if(IsIntersecting(vb, right))
    //             {
    //                 inter_points.push_back(LineSegIntersect(vb, right));
    //             }
    //             if(IsIntersecting(vb, up))
    //             {
    //                 inter_points.push_back(LineSegIntersect(vb, up));
    //             }
    //             if(IsIntersecting(vb, down))
    //             {
    //                 inter_points.push_back(LineSegIntersect(vb, down));
    //             }
    //             if(edges[i]->ori_vertex)
    //             {

    //                 auto ori_point = edges[i]->ori_vertex->coor;
    //                 // std::cout<<site_points [ edge_to_decisive_point[i](0)].transpose()<<std::endl;
    //                 // std::cout<<site_points [ edge_to_decisive_point[i](1)].transpose()<<std::endl;
    //                 // std::cout<<ori_point.transpose()<<std::endl;
    //                 // std::cout<<vb.n.transpose()<<std::endl;
    //                 // std::cout<<"---"<<std::endl;
    //                 for(size_t j = 0; j != inter_points.size(); ++j)
    //                 {
    //                     if(CheckPointToLine(ori_point, inter_points[j], site_point) > 0)
    //                     {
    //                         vertices.push_back(new HEVertex(inter_points[j]));
    //                         vertices.back()->id = vertices.size() - 1;
    //                         edges[i]->des_vertex = vertices.back() ;
    //                         edges[i]->twin_edge->ori_vertex = vertices.back();
    //                         break;
    //                     }
    //                 }
    //                 if(edges[i]->des_vertex == nullptr) std::cout<<"Not assign a vertex."<<std::endl;
    //             }
    //             else
    //             {
    //                 auto des_point = edges[i]->des_vertex->coor;
    //                 for(size_t j = 0; j != inter_points.size(); ++j)
    //                 {
    //                     if(CheckPointToLine(inter_points[j], des_point, site_point) > 0)
    //                     {
    //                         vertices.push_back(new HEVertex(inter_points[j]));
    //                         vertices.back()->id = vertices.size() - 1;
    //                         edges[i]->ori_vertex = vertices.back();
    //                         edges[i]->twin_edge->des_vertex = vertices.back();
    //                         break;
    //                     }
    //                 }
    //                 if(edges[i]->ori_vertex == nullptr) std::cout<<"Not assign a vertex."<<std::endl;
    //             }
    //         }
    //         else
    //         {
    //             std::cout<<RED<<"[ERROR]::[BBTruncation]::Face id is -1."<<RESET<<std::endl;
    //             continue;
    //         }
    //     }
    // }

    // edge a split edge b, intersection v
    HEEdge * EdgeSplitEdge(HEEdge *b, HEEdge * a, HEVertex * v, bool ori)
    {
        HEEdge *e = new HEEdge(v, b->des_vertex);
        if(ori)
        {

            a->ori_vertex = v;
            a->twin_edge->des_vertex = v;

            b->des_vertex = v;
            e->next_edge = b->next_edge;
            e->next_edge->pre_edge = e;

            b->next_edge = a;
            a->pre_edge = b;

            e->pre_edge = a->twin_edge;
            a->twin_edge->next_edge = e;
            
            e->parent_face = a->twin_edge->parent_face;
            b->parent_face = a->parent_face;
            
            
        }
        else
        {
            a->des_vertex = v;
            a->twin_edge->ori_vertex = v;

            b->des_vertex = v;
            e->next_edge = b->next_edge;
            e->next_edge->pre_edge = e;

            b->next_edge = a->twin_edge;
            a->twin_edge->pre_edge = b;

            e->pre_edge = a;
            a->next_edge = e;   

            e->parent_face = a->parent_face;
            b->parent_face = a->twin_edge->parent_face;
        }
        return e;
    }
    void Voronoi2D::BBTruncation(const BoundingBox &bb)
    {
        // this file is to use boundingbox to truncate the edges, which has a nullptr vertex
        // auto &faces = he.faces;
        if(site_points.size()==0) return;
        auto &edges = he.edges;
        auto &vertices = he.vertices;
        Point2 left_top(bb.x_min, bb.y_max), left_bottom(bb.x_min, bb.y_min);
        Point2 right_bottom(bb.x_max, bb.y_min), right_top(bb.x_max, bb.y_max);
        std::vector<bool> proccessed_edges(edges.size(), false);
        // clockwise
        HEVertex *left_top_ptr = new HEVertex(left_top);
        HEVertex *right_top_ptr = new HEVertex(right_top);
        HEVertex *right_bottom_ptr = new HEVertex(right_bottom);
        HEVertex *left_bottom_ptr = new HEVertex(left_bottom);
        //left_top_ptr->id = vertices.size();
        vertices.push_back(left_top_ptr);
        //right_top_ptr->id = vertices.size();
        vertices.push_back(right_top_ptr);
        //right_bottom_ptr->id = vertices.size();
        vertices.push_back(right_bottom_ptr);
        //left_bottom_ptr->id = vertices.size();
        vertices.push_back(left_bottom_ptr);


        size_t border_entry = edges.size();//edge id start from border_entry is border.
        // Add four edge
        HEEdge *top_edge = new HEEdge(left_top_ptr, right_top_ptr);
        top_edge->id = edges.size();
        edges.push_back(top_edge);

        HEEdge *right_edge = new HEEdge(right_top_ptr, right_bottom_ptr);
        right_edge->id = edges.size();
        edges.push_back(right_edge);

        HEEdge *bottom_edge = new HEEdge(right_bottom_ptr, left_bottom_ptr);
        bottom_edge->id = edges.size();
        edges.push_back(bottom_edge);

        HEEdge *left_edge = new HEEdge(left_bottom_ptr, left_top_ptr);
        left_edge->id = edges.size();
        edges.push_back(left_edge);
        // connect these edges
        top_edge->next_edge = right_edge;
        top_edge->pre_edge = left_edge;

        right_edge->next_edge = bottom_edge;
        right_edge->pre_edge = top_edge;

        bottom_edge->next_edge = left_edge;
        bottom_edge->pre_edge = right_edge;

        left_edge->next_edge = top_edge;
        left_edge->pre_edge = bottom_edge;

        // LineSegment left(left_top, left_bottom), right(right_top, right_bottom);
        // LineSegment up(left_top, right_top), down(left_bottom, right_bottom);
        // use border to truncate the edges
        for(size_t i = 0; i != border_entry; ++i)
        {
            if(proccessed_edges[i]) continue;
            proccessed_edges[i] = true;
            proccessed_edges[edges[i]->twin_edge->id] = true;
            int fid = edges[i]->parent_face->id;
            if(fid == -1) 
            {
                std::cout<<YELLOW<<"[WARNING]::[BBTruncation]::Invalid face id."<<RESET<<std::endl;
                continue;                
            }
            Point2 site_point = site_points[fid];
            if(!edges[i]->ori_vertex && !edges[i]->des_vertex )
            {
                std::cout<<YELLOW<<"[WARNING]::[BBTruncation]::Edge with two nullptr vertices is abnormal."<<RESET<<std::endl;
                continue;
                //exit(0);
            }
            else if(edges[i]->ori_vertex && edges[i]->des_vertex)
            {
                LineSegment edge_seg(edges[i]->ori_vertex->coor, edges[i]->des_vertex->coor);
                bool ori_inside = bb.IsInside(edges[i]->ori_vertex->coor);
                bool des_inside = bb.IsInside(edges[i]->des_vertex->coor);
                if(ori_inside && des_inside) continue;
                std::vector<std::pair<geometry::Vector2, int>> inter_points;
                for(size_t eid = border_entry; eid < edges.size(); ++eid)
                {
                    LineSegment border_seg(edges[eid]->ori_vertex->coor, edges[eid]->des_vertex->coor);
                    
                    if(IsIntersecting(border_seg, edge_seg))
                    {
                        geometry::Point2 point = SegIntersect(border_seg, edge_seg);
                        auto item = std::make_pair(point,eid);
                        inter_points.push_back(item);
                        // break;
                    }
                }
                std::sort(inter_points.begin(), inter_points.end(), LessOfIntersection);
                auto tmp_inter_points = inter_points;
                inter_points.clear();
                for(size_t i = 0; i != tmp_inter_points.size(); ++i)
                {

                    if(inter_points.size() == 0 || 
                        geometry::Distance(tmp_inter_points[i].first, inter_points.back().first) > EPS)
                    {
                        inter_points.push_back(tmp_inter_points[i]);
                    }
                }
                if(inter_points.size() == 0) 
                {
                    std::cout<<BLUE<<"[INFO]::[BBTruncation]::Cannot find the intersection, and this edge will be deleted from DCEL."<<RESET<<std::endl;
                    edges[i]->id = -1;
                    edges[i]->twin_edge->id = -1;
                    edges[i]->ori_vertex->id = -1;
                    edges[i]->des_vertex->id = -1;
                    continue;
                }
                else if(inter_points.size() == 2)
                {
                    // std::cout<<YELLOW<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<RESET<<std::endl;
                    edges[i]->ori_vertex->id = -1;
                    edges[i]->des_vertex->id = -1;
                    HEVertex *v0 = nullptr;
                    HEVertex *v1 = nullptr;
                    int eid0 = -1;
                    int eid1 = -1;
                    if(CheckPointToLine(inter_points[0].first, inter_points[1].first, site_point) < 0)
                    {
                        //0 ori, 1 des
                        v0 = new HEVertex (inter_points[0].first);
                        v1 = new HEVertex (inter_points[1].first);
                        eid0 = inter_points[0].second;
                        eid1 = inter_points[1].second;

                    }
                    else
                    {
                        //0 des, 1 ori
                        v0 = new HEVertex (inter_points[1].first);
                        v1 = new HEVertex (inter_points[0].first);
                        eid0 = inter_points[1].second;
                        eid1 = inter_points[0].second;
                    }
                    HEEdge *e0 = EdgeSplitEdge(edges[eid0], edges[i], v0, true);
                    HEEdge *e1 = EdgeSplitEdge(edges[eid1], edges[i], v1, false);
                    e0->id = edges.size();
                    edges.push_back(e0);
                    e1->id = edges.size();
                    edges.push_back(e1);
                }
                else if(inter_points.size() == 1)
                {
                    
                    HEVertex *v = new HEVertex (inter_points[0].first);
                    vertices.push_back(v);
                    int inter_eid = inter_points[0].second;
                    if(!ori_inside)
                    {
                        edges[i]->ori_vertex->id = -1;
                        HEEdge *e = EdgeSplitEdge(edges[inter_eid], edges[i], v, true);
                        e->id = edges.size();
                        edges.push_back(e);
                        // HEEdge *e = new HEEdge(v, edges[inter_eid]->des_vertex);
                        // edges[inter_eid]->des_vertex = v;

                        // e->next_edge = edges[inter_eid]->next_edge;
                        // e->next_edge->pre_edge = e;

                        // edges[inter_eid]->next_edge = edges[i];
                        // edges[i]->pre_edge = edges[inter_eid];

                        // e->pre_edge = edges[i]->twin_edge;
                        // edges[i]->twin_edge->next_edge = e;
                    }
                    else if(!des_inside)
                    {
                        edges[i]->des_vertex->id = -1;
                        HEEdge *e = EdgeSplitEdge(edges[inter_eid], edges[i], v, false);
                        e->id = edges.size();
                        edges.push_back(e);
                        // HEEdge *e = new HEEdge(v, edges[inter_eid]->des_vertex);
                        // edges[inter_eid]->des_vertex = v;

                        // e->next_edge = edges[inter_eid]->next_edge;
                        // e->next_edge->pre_edge = e;

                        // edges[inter_eid]->next_edge = edges[i]->twin_edge;
                        // edges[i]->twin_edge->pre_edge = edges[inter_eid];

                        // e->pre_edge = edges[i];
                        // edges[i]->next_edge = e;                        
                    }
                }
                else
                {
                    std::cout<<YELLOW<<"[WARNING]::[BBTruncation]::Special case: more than 2 intersection points."<<RESET<<std::endl;
                    edges[i]->id = -1;
                    edges[i]->twin_edge->id = -1;
                    edges[i]->ori_vertex->id = -1;
                    edges[i]->des_vertex->id = -1;
                    continue;                    
                }
            }
            else 
            {
                // if((edges[i]->ori_vertex && !bb.IsInside(edges[i]->ori_vertex->coor)) || 
                //     (edges[i]->des_vertex && !bb.IsInside(edges[i]->des_vertex->coor)))
                // {
                //     edges[i]->id = -1;
                //     edges[i]->twin_edge->id = -1;
                //     continue;
                // }
                // std::cout<<"???"<< std::endl;

                Line vb = VerticalBisector(site_points [ edge_to_decisive_point[i](0)], site_points[edge_to_decisive_point[i](1)]);
                std::vector<std::pair<geometry::Vector2, int>> inter_points;
                for(size_t eid = border_entry; eid < edges.size(); ++eid)
                {
                    LineSegment border_seg(edges[eid]->ori_vertex->coor, edges[eid]->des_vertex->coor);
                    
                    if(IsIntersecting(vb, border_seg))
                    {
                        geometry::Point2 point = LineSegIntersect(vb, border_seg);
                        auto item = std::make_pair(point,eid);
                        inter_points.push_back(item);
                        // break;
                    }
                }
                std::sort(inter_points.begin(), inter_points.end(), LessOfIntersection);
                auto tmp_inter_points = inter_points;
                inter_points.clear();           
                for(size_t i = 0; i != tmp_inter_points.size(); ++i)
                {

                    if(inter_points.size() == 0 || 
                        geometry::Distance(tmp_inter_points[i].first, inter_points.back().first) > EPS)
                    {
                        // if(inter_points.size() != 0) std::cout<<"Distance: "<<tmp_inter_points[i].first.transpose()<<"\n"<<inter_points.back().first.transpose()<<std::endl;
                        inter_points.push_back(tmp_inter_points[i]);
                    }
                }
                if(inter_points.size() == 0)
                {
                    std::cout<<YELLOW<<"[WARNING]::[BBTruncation]::Cannot find the intersection (with one end)."<<RESET<<std::endl;
                    edges[i]->id = -1;
                    edges[i]->twin_edge->id = -1;
                    continue;                    
                }
                else if(inter_points.size() == 2)
                {

                    if(edges[i]->ori_vertex) 
                    {
                        size_t inter_pid = 0;
                        for(; inter_pid != inter_points.size(); ++inter_pid)
                        {
                            if(CheckPointToLine(edges[i]->ori_vertex->coor, inter_points[inter_pid].first, site_point) < 0)
                            {
                                break;
                            }
                        }
                        if(!bb.IsInside(edges[i]->ori_vertex->coor))
                        {
                            edges[i]->ori_vertex->id = -1;
                            // no intersection with border.
                            if(inter_pid == inter_points.size())
                            {
                                edges[i]->id = -1;
                                edges[i]->twin_edge->id = -1;             
                                continue;                
                            }
                            else
                            {
                                
                                if(CheckPointToLine(inter_points[0].first, inter_points[1].first, site_point) < 0)
                                {
                                    HEVertex *v0 = new HEVertex (inter_points[0].first);
                                    HEVertex *v1 = new HEVertex (inter_points[1].first);
                                    int eid0 = inter_points[0].second;
                                    int eid1 = inter_points[1].second;
                                    HEEdge *e0 = EdgeSplitEdge(edges[eid0], edges[i], v0, true);
                                    HEEdge *e1 = EdgeSplitEdge(edges[eid1], edges[i], v1, false);
                                    e0->id = edges.size();
                                    edges.push_back(e0);
                                    e1->id = edges.size();
                                    edges.push_back(e1);  
                                }
                                else
                                {
                                    HEVertex *v0 = new HEVertex (inter_points[1].first);
                                    HEVertex *v1 = new HEVertex (inter_points[0].first);
                                    int eid0 = inter_points[1].second;
                                    int eid1 = inter_points[0].second;
                                    HEEdge *e0 = EdgeSplitEdge(edges[eid0], edges[i], v0, true);
                                    HEEdge *e1 = EdgeSplitEdge(edges[eid1], edges[i], v1, false);
                                    e0->id = edges.size();
                                    edges.push_back(e0);
                                    e1->id = edges.size();
                                    edges.push_back(e1);  
                                }
                              
                            }
                        }
                        else
                        {
                            size_t inter_eid = inter_points[inter_pid].second;
                            HEVertex *v = new HEVertex (inter_points[inter_pid].first);
                            vertices.push_back(v);
                            
                            HEEdge *e = EdgeSplitEdge(edges[inter_eid], edges[i], v, false);
                            // std::cout<<"??????0---"<< edges[i]->id <<" "<<edges[i]->pre_edge->id<< std::endl;
                            e->id = edges.size();
                            edges.push_back(e);
                            // std::cout<< e->id <<" "<<e->next_edge->id<< std::endl;
                            // std::cout<< edges[inter_eid]->id <<" "<<edges[inter_eid]->next_edge->id<< std::endl;
                        }
                    }
                    else
                    {
                        size_t inter_pid = 0;
                        for(; inter_pid != inter_points.size(); ++inter_pid)
                        {
                            if(CheckPointToLine(inter_points[inter_pid].first, edges[i]->des_vertex->coor, site_point) < 0)
                            {
                                break;
                            }
                        }
                        if(!bb.IsInside(edges[i]->des_vertex->coor))
                        {
                            edges[i]->des_vertex->id = -1;   
                            if(inter_pid == inter_points.size())
                            {
                                edges[i]->id = -1;
                                edges[i]->twin_edge->id = -1;   
                                          
                                continue;                
                            }
                            else
                            {
                                if(CheckPointToLine(inter_points[0].first, inter_points[1].first, site_point) < 0)
                                {
                                    HEVertex *v0 = new HEVertex (inter_points[0].first);
                                    HEVertex *v1 = new HEVertex (inter_points[1].first);
                                    int eid0 = inter_points[0].second;
                                    int eid1 = inter_points[1].second;
                                    HEEdge *e0 = EdgeSplitEdge(edges[eid0], edges[i], v0, true);
                                    HEEdge *e1 = EdgeSplitEdge(edges[eid1], edges[i], v1, false);
                                    e0->id = edges.size();
                                    edges.push_back(e0);
                                    e1->id = edges.size();
                                    edges.push_back(e1);  
                                }
                                else
                                {
                                    HEVertex *v0 = new HEVertex (inter_points[1].first);
                                    HEVertex *v1 = new HEVertex (inter_points[0].first);
                                    int eid0 = inter_points[1].second;
                                    int eid1 = inter_points[0].second;
                                    HEEdge *e0 = EdgeSplitEdge(edges[eid0], edges[i], v0, true);
                                    HEEdge *e1 = EdgeSplitEdge(edges[eid1], edges[i], v1, false);
                                    e0->id = edges.size();
                                    edges.push_back(e0);
                                    e1->id = edges.size();
                                    edges.push_back(e1);  
                                }
                              
                            }
                        }
                        else
                        {
                            // std::cout<<"??????1---"<< edges[i]->id <<" "<<edges[i]->next_edge->id<< std::endl;
                            size_t inter_eid = inter_points[inter_pid].second;
                            HEVertex *v = new HEVertex (inter_points[inter_pid].first);
                            vertices.push_back(v);
                            HEEdge *e = EdgeSplitEdge(edges[inter_eid], edges[i], v, true);
                            
                            e->id = edges.size();
                            edges.push_back(e);
                            // std::cout<< e->id <<" "<<e->next_edge->id<< std::endl;
                            // std::cout<< edges[inter_eid]->id <<" "<<edges[inter_eid]->next_edge->id<< std::endl;
                        }
                    }
                }
                else
                {
                    
                    std::cout<<YELLOW<<"[WARNING]::[BBTruncation]::Special case: more than 2 intersection points. "<<inter_points.size()<<RESET<<std::endl;
                    edges[i]->id = -1;
                    edges[i]->twin_edge->id = -1;
                    continue;                                
                }
            }
        }
        std::cout<<"Finish truncation."<< std::endl;
        he.RearrangeFaceIncEdge();      
        std::cout<<"Finish Rearrangement."<< std::endl;  
    }
    void Voronoi2D::GenerateDiagram()
    {
        if(site_points.size()==0) return;
        std::priority_queue<SiteEvent *, std::vector<SiteEvent *>, GreaterSiteEvent>().swap(site_events);
        std::priority_queue<CircleEvent *, std::vector<CircleEvent *>, GreaterCircleEvent>().swap(circle_events);
        for(size_t i = 0; i != site_points.size(); ++i)
        {
            //add site events
            SiteEvent *tmp_se = new SiteEvent(site_points[i](0), site_points[i](1), i);
            site_events.push(tmp_se);
        }
        //max number of voronoi edges: 3n-6
        //max number of voronoi vertex: 2n-5
        //max number of voronoi face: n
        he.vertices.reserve(2 * site_points.size());
        he.edges.reserve(3 * site_points.size() * 2);
        he.faces.resize(site_points.size(), nullptr);
        for(size_t i = 0; i != site_points.size(); ++i)
        {
            he.faces[i] = new HEFace();
            he.faces[i]->id = i;
        }
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
        // BBTruncation();
        std::cout<<"voronoi: "<<he.faces.size()<<" faces, "<<he.edges.size()<<" edges, "<<he.vertices.size()<<" vertices."<<std::endl;
    }
}
}