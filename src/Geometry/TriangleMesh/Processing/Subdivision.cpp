#include "Subdivision.h"

namespace dragon
{
namespace geometry
{
namespace mesh
{
    std::shared_ptr<TriangleMesh> LoopSubdivision(const TriangleMesh &mesh, int iteration)
    {
        HalfEdge he;
        he.FromTriangleMesh(mesh);
        he.CheckBorder();
        int iter = 0;
        
        auto &vertices = he.vertices;
        auto &edges = he.edges;
        auto &faces = he.faces;
        auto &is_border = he.is_border;
        std::vector<int> edge_to_np(edges.size(), -1);       
        std::vector<int> edge_to_ne(edges.size(), -1); 
        while(iter < iteration)
        {
            iter ++ ;
            // add the new points
            size_t old_vertex_n = vertices.size();
            size_t old_face_n = faces.size();
            size_t old_edge_n = edges.size();
            for(size_t i = 0; i != edges.size(); ++i)
            {
                // to avoid duplicated vertices
                if(edge_to_np[i] != -1) continue;
                HEVertex *nv = new HEVertex();
                if(he.has_colors) nv->color = (edges[i]->des_vertex->color + edges[i]->ori_vertex->color) * 0.5;
                nv->id = vertices.size();
                edge_to_np[i] = vertices.size();
                if(edges[i]->twin_edge == nullptr)
                {
                    nv->coor = (edges[i]->des_vertex->coor + edges[i]->ori_vertex->coor) * 0.5;
                }
                else
                {
                    edge_to_np[edges[i]->twin_edge->id] = vertices.size();
                    nv->coor = 0.375 * (edges[i]->des_vertex->coor + edges[i]->ori_vertex->coor)
                        + 0.125 *(edges[i]->next_edge->des_vertex->coor + edges[i]->twin_edge->next_edge->des_vertex->coor);
                }
                vertices.push_back(nv);
            }
            // move old points
            for(size_t i = 0; i != old_vertex_n; ++i)
            {
                // if the vertex is border, we will not move its position
                if(is_border[i]) continue;
                auto start_edge = vertices[i]->inc_edge;
                auto tmp_edge = start_edge;
                int neighbor_num = 0;
                double u = 0.1875;
                geometry::Point3 neighbor_sum = geometry::Point3::Zero();
                do
                {
                    neighbor_num += 1;
                    neighbor_sum += tmp_edge->des_vertex->coor;
                    tmp_edge = tmp_edge->twin_edge->next_edge;
                } while (tmp_edge != start_edge);
                if(neighbor_num > 3) u = 0.375;    
                vertices[i]->coor = vertices[i]->coor * (1 - u) + u * neighbor_sum / neighbor_num; 
            }

            // reconnect the points
            
            for(size_t i = 0; i != old_face_n; ++i)
            {
                // for each face, 3 new faces and 9 new edges will be generated
                auto oe1 = faces[i]->inc_edge;
                auto oe2 = faces[i]->inc_edge->next_edge;
                auto oe3 = faces[i]->inc_edge->pre_edge;

                HEEdge *e1 = new HEEdge(vertices[edge_to_np[oe1->id]], oe1->des_vertex);
                e1->id = edges.size();
                edges.push_back(e1);
                if(vertices[edge_to_np[oe1->id]]->inc_edge == nullptr) vertices[edge_to_np[oe1->id]]->inc_edge = e1;
                edge_to_ne[oe1->id] = e1->id;

                HEEdge *e2 = new HEEdge(vertices[edge_to_np[oe2->id]], oe2->des_vertex);
                e2->id = edges.size();
                edges.push_back(e2);
                if(vertices[edge_to_np[oe2->id]]->inc_edge == nullptr) vertices[edge_to_np[oe2->id]]->inc_edge = e2;
                edge_to_ne[oe2->id] = e2->id;
                
                HEEdge *e3 = new HEEdge(vertices[edge_to_np[oe3->id]], oe3->des_vertex);
                e3->id = edges.size();
                edges.push_back(e3);
                if(vertices[edge_to_np[oe3->id]]->inc_edge == nullptr) vertices[edge_to_np[oe3->id]]->inc_edge = e3;
                edge_to_ne[oe3->id] = e3->id;

                HEEdge *e4 = new HEEdge(vertices[edge_to_np[oe1->id]], vertices[edge_to_np[oe2->id]]);
                e4->id = edges.size();
                edges.push_back(e4);
                HEEdge *e5 = new HEEdge(vertices[edge_to_np[oe2->id]], vertices[edge_to_np[oe3->id]]);
                e5->id = edges.size();
                edges.push_back(e5);
                HEEdge *e6 = new HEEdge(vertices[edge_to_np[oe3->id]], vertices[edge_to_np[oe1->id]]);
                e6->id = edges.size();
                edges.push_back(e6);

                HEEdge *e7 = new HEEdge(vertices[edge_to_np[oe2->id]], vertices[edge_to_np[oe1->id]]);
                e7->id = edges.size();
                edges.push_back(e7);
                HEEdge *e8 = new HEEdge(vertices[edge_to_np[oe3->id]], vertices[edge_to_np[oe2->id]]);
                e8->id = edges.size();
                edges.push_back(e8);
                HEEdge *e9 = new HEEdge(vertices[edge_to_np[oe1->id]], vertices[edge_to_np[oe3->id]]);
                e9->id = edges.size();
                edges.push_back(e9);

                e4->twin_edge = e7;
                e7->twin_edge = e4;
                e5->twin_edge = e8;
                e8->twin_edge = e5;
                e6->twin_edge = e9;
                e9->twin_edge = e6;
                
                oe1->des_vertex = vertices[edge_to_np[oe1->id]];
                oe2->des_vertex = vertices[edge_to_np[oe2->id]];
                oe3->des_vertex = vertices[edge_to_np[oe3->id]];

                //
                oe1->next_edge = e9;
                e9->pre_edge = oe1;
                e3->next_edge = oe1;
                oe1->pre_edge = e3;
                e9->next_edge = e3;
                e3->pre_edge = e9;

                oe2->next_edge = e7;
                e7->pre_edge = oe2;
                e1->next_edge = oe2;
                oe2->pre_edge = e1;
                e7->next_edge = e1;
                e1->pre_edge = e7;

                oe3->next_edge = e8;
                e8->pre_edge = oe3;
                e2->next_edge = oe3;
                oe3->pre_edge = e2;
                e8->next_edge = e2;
                e2->pre_edge = e8;

                e4->next_edge = e5;
                e5->pre_edge = e4;
                e5->next_edge = e6;
                e6->pre_edge = e5;
                e6->next_edge = e4;
                e4->pre_edge = e6;

                faces[i]->inc_edge = e4;

                HEFace *f1 = new HEFace(oe1);
                HEFace *f2 = new HEFace(oe2);
                HEFace *f3 = new HEFace(oe3);

                f1->id = faces.size();
                faces.push_back(f1);
                f2->id = faces.size();
                faces.push_back(f2);
                f3->id = faces.size();
                faces.push_back(f3);
            }
            // there is no need to maintain the twin relationship
            for(size_t i = 0; i != old_edge_n; ++i)
            {
                if(edge_to_ne[i] == -1) continue;
                if(edges[i]->twin_edge == nullptr) continue;
                auto old_twin = edges[i]->twin_edge;
                edges[i]->twin_edge = edges[edge_to_ne[old_twin->id]];
                edges[edge_to_ne[old_twin->id]]->twin_edge = edges[i];

                old_twin->twin_edge = edges[edge_to_ne[i]];
                edges[edge_to_ne[i]]->twin_edge = old_twin;
                edge_to_ne[i] = -1;
                edge_to_ne[old_twin->id] = -1;
            }
        }
        TriangleMesh _mesh;
        he.ToTriangleMesh(_mesh);
        return std::make_shared<TriangleMesh>(_mesh);
    }
}
}
}