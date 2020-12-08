#include "HalfEdge.h"
#include <unordered_map>

namespace dragon
{
namespace geometry
{
    void HalfEdge::FromTriangleMesh(const geometry::TriangleMesh &mesh)
    {
        std::unordered_map<std::pair<size_t, size_t>, HEEdge *, PairHasher> visited_edges; 
        has_colors = mesh.HasColors();
        vertices.resize(mesh.points.size());
        for(size_t i = 0; i != mesh.points.size(); ++i)
        {
            vertices[i] = HEVertex(mesh.points[i]);
            vertices[i].id = i;
            if(has_colors) vertices[i].color = mesh.colors[i];
        }
        edges.resize(mesh.triangles.size() * 3);
        faces.resize(mesh.triangles.size());
        //currently we don't consider to reverse the triangle connection order.
        for(size_t i = 0; i != mesh.triangles.size(); ++i)
        {
            //create edges and face
            // if(i%10000==0) std::cout<<i<<std::endl;
            size_t vid0 = mesh.triangles[i](0);
            size_t vid1 = mesh.triangles[i](1);
            size_t vid2 = mesh.triangles[i](2);
            HEEdge * edge0 = & edges[i * 3 + 0];
            *edge0 = HEEdge(&vertices[vid0], &vertices[vid1]);
            if(vertices[vid0].inc_edge == nullptr)
            {
                vertices[vid0].inc_edge = edge0;
            }       
            HEEdge * edge1 = & edges[i * 3 + 1];
            *edge1 = HEEdge(&vertices[vid1], &vertices[vid2]);
            if(vertices[vid1].inc_edge == nullptr)
            {
                vertices[vid1].inc_edge = edge1;
            }          
            HEEdge * edge2 = & edges[i * 3 + 2];
            *edge2 = HEEdge(&vertices[vid2], &vertices[vid0]);
            if(vertices[vid2].inc_edge == nullptr)
            {
                vertices[vid2].inc_edge = edge2;
            }  


            edge0->next_edge = edge1;
            edge0->pre_edge = edge2;

            edge1->next_edge = edge2;
            edge1->pre_edge = edge0;

            edge2->next_edge = edge0;
            edge2->pre_edge = edge1;
            // finish creating a face.
            // connect with other fragment
            
            // process edges
            std::pair<size_t, size_t> edge_id_0 = std::make_pair(vid0, vid1);
            std::pair<size_t, size_t> edge_id_0_r = std::make_pair(vid1, vid0);
            std::pair<size_t, size_t> edge_id_1 = std::make_pair(vid1, vid2);
            std::pair<size_t, size_t> edge_id_1_r = std::make_pair(vid2, vid1);
            std::pair<size_t, size_t> edge_id_2 = std::make_pair(vid2, vid0);
            std::pair<size_t, size_t> edge_id_2_r = std::make_pair(vid0, vid2);
            if(visited_edges.find(edge_id_0) != visited_edges.end())
            {
                //error, not maniford
                std::cout<<YELLOW<<"[WARNING]::[HalfEdge]::Not a maniford."<<RESET<<std::endl;
                continue;
            }            
            else if(visited_edges.find(edge_id_0_r) != visited_edges.end())
            {
                // connect
                visited_edges[edge_id_0_r]->twin_edge = edge0;
                edge0->twin_edge = visited_edges[edge_id_0_r];
            }
            visited_edges[edge_id_0] = edge0;

            if(visited_edges.find(edge_id_1) != visited_edges.end())
            {
                //error, not maniford
                std::cout<<YELLOW<<"[WARNING]::[HalfEdge]::Not a maniford."<<RESET<<std::endl;
                continue;
            }            
            else if(visited_edges.find(edge_id_1_r) != visited_edges.end())
            {
                // connect
                visited_edges[edge_id_1_r]->twin_edge = edge1;
                edge1->twin_edge = visited_edges[edge_id_1_r];
            }
            visited_edges[edge_id_1] = edge1;

            if(visited_edges.find(edge_id_2) != visited_edges.end())
            {
                //error, not maniford
                std::cout<<YELLOW<<"[WARNING]::[HalfEdge]::Not a maniford."<<RESET<<std::endl;
                continue;
            }            
            else if(visited_edges.find(edge_id_2_r) != visited_edges.end())
            {
                // connect
                visited_edges[edge_id_2_r]->twin_edge = edge2;
                edge2->twin_edge = visited_edges[edge_id_2_r];
            }

            visited_edges[edge_id_2] = edge2;
            HEFace * new_face = & faces[i];
            *new_face = HEFace(edge0);
            new_face->id = i;
            edge0->parent_face = new_face;
            edge1->parent_face = new_face; 
            edge2->parent_face = new_face;
        }
    }
    void HalfEdge::ToTriangleMesh(TriangleMesh &mesh)
    {
        std::vector<int> visited_vertices(vertices.size(), -1);
        size_t pos = 0;
        for(size_t i = 0; i != faces.size(); ++i)
        {
            HEEdge *start_edge = faces[i].inc_edge;
            if(start_edge == nullptr) continue;
            if(visited_vertices[start_edge->ori_vertex->id] == -1)
                visited_vertices[start_edge->ori_vertex->id] = pos++;
            if(visited_vertices[start_edge->des_vertex->id] == -1)
                visited_vertices[start_edge->des_vertex->id] = pos++;
            if(visited_vertices[start_edge->next_edge->des_vertex->id] == -1)
                visited_vertices[start_edge->next_edge->des_vertex->id] = pos ++;
        }
        mesh.Reset();
        mesh.points.resize(pos);
        mesh.triangles.resize(faces.size());
        for(size_t i = 0; i != faces.size(); ++i)
        {
            HEEdge *start_edge = faces[i].inc_edge;
            if(start_edge == nullptr) continue;
            size_t vid0 = visited_vertices[start_edge->ori_vertex->id];
            size_t vid1 = visited_vertices[start_edge->des_vertex->id];
            size_t vid2 = visited_vertices[start_edge->next_edge->des_vertex->id];
            mesh.triangles[i] = geometry::Point3ui(vid0, vid1, vid2);
        }
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            if(visited_vertices[i] != -1)
            {
                mesh.points[visited_vertices[i]] = vertices[i].coor; 
            }
        }
        if(has_colors)
        {
            mesh.colors.resize(pos);
            for(size_t i = 0; i != vertices.size(); ++i)
            {
                if(visited_vertices[i] != -1)
                {
                    mesh.colors[visited_vertices[i]] = vertices[i].color; 
                }
            }
        }
    }

    void HalfEdge::CheckBorder()
    {
        is_border.resize(vertices.size(), false);
        for(size_t i = 0; i != edges.size(); ++i)
        {   
            //std::cout<<i<<" "<<(edges[i]->ori_vertex->id)<<std::endl;
            if(edges[i].twin_edge == nullptr)
            {
                is_border[edges[i].ori_vertex->id] = true;
                is_border[edges[i].des_vertex->id] = true;
            }
        }
    }
}
}