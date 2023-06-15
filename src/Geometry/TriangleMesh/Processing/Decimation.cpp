#include "Decimation.h"
#include <set>
namespace dragon
{
namespace geometry
{
namespace mesh
{
    Vector4 zero_one = Vector4(0, 0, 0, 1);
    std::tuple<Point3, double> Decimator::ComputeEdgeError(int eid)
    {
        Point3 np, v1, v2;
        double e;
        int vid1 = he.edges[eid]->ori_vertex->id;
        v1 = he.vertices[vid1]->coor;
        int vid2 = he.edges[eid]->des_vertex->id;
        v2 = he.vertices[vid2]->coor;

        bool is_border = he.is_border[vid1] & he.is_border[vid2];
        Matrix4 q_e = q_mat[vid1] + q_mat[vid2];
        Matrix4 coe = q_e;
        coe.block<1, 4>(3, 0) = zero_one.transpose();
        if(!is_border && coe.fullPivLu().isInvertible())
        {
            auto homo_new_p = coe.inverse() * zero_one;
            np = homo_new_p.head<3>();
            e = homo_new_p.transpose() * q_e * homo_new_p;
        }
        else
        {
            // std::cout<<coe<<std::endl<<std::endl;
            geometry::Vector4 homo_v1 = geometry::Vector4(v1(0), v1(1), v1(2),1);
            geometry::Vector4 homo_v2 = geometry::Vector4(v2(0), v2(1), v2(2),1);
            geometry::Vector4 homo_v12 =( homo_v1 + homo_v2 )/2;

            double error_i = (homo_v1.transpose() * q_e * homo_v1);
            double error_j = (homo_v2.transpose() * q_e * homo_v2);
            double error_ij = (homo_v12.transpose() * q_e * homo_v12);
            // std::cout<<error_i<<" "<<error_j<<" "<<error_ij<<std::endl;
            e = std::min(error_ij, std::min(error_i, error_j));
            if(e == error_i) np = v1;
            else if(e == error_j) np = v2;
            else np = homo_v12.head<3>();
        }        
        return std::make_tuple(np, e);
    }
    void Decimator::QuadricConfig()
    {
        he.CheckBorder();
        auto &vertices = he.vertices;
        auto &edges = he.edges;
        auto &faces = he.faces;
        current_triangle_num = faces.size();
        q_mat.clear();
        q_mat.resize(vertices.size(), Matrix4::Zero());
        planes.clear();
        planes.resize(faces.size());
        dirty.clear();
        dirty.resize(faces.size(), false);
        vertex_to_faces.clear();
        vertex_to_faces.resize(vertices.size(), std::vector<HEFace *>());
        updated.clear();
        updated.resize(edges.size(), false);        
        // edge error will be compacted in edge_weight
        // compute the plane for each face.
        for(size_t i = 0; i != faces.size(); ++i)
        {
            if(IsValid(faces[i]))
            {   
                int vid1 = faces[i]->inc_edge->ori_vertex->id;
                int vid2 = faces[i]->inc_edge->des_vertex->id;
                int vid3 = faces[i]->inc_edge->next_edge->des_vertex->id;

                Point3 p1 = vertices[vid1]->coor; 
                Point3 p2 = vertices[vid2]->coor;
                Point3 p3 = vertices[vid3]->coor;
                Point3 n = ((p2 - p1).cross(p3 - p1));
                n.normalize();
                // if(n(2) < 0)
                // {
                //     std::cout<<faces[i]->id<<" "<<"???"<<std::endl;
                // }
        // if(std::isnan(n(0)))
        // std::cout<<n.transpose()<<" \n"<<p1.transpose()<<" \n"<<p2.transpose()<<" \n"<<p3.transpose()<<std::endl;
                planes[i].block<3, 1>(0, 0) = n;
                planes[i](3) = -n.dot(p1);
                vertex_to_faces[vid1].push_back(faces[i]);
                vertex_to_faces[vid2].push_back(faces[i]);
                vertex_to_faces[vid3].push_back(faces[i]);
            }
        }
        // compute Q matrix for vertices
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            for(size_t j = 0; j != vertex_to_faces[i].size(); ++j)
            q_mat[i] += planes[vertex_to_faces[i][j]->id] * planes[vertex_to_faces[i][j]->id].transpose();
        }
        // compute the new position and error for each edge
        // put the edge into queue
        std::priority_queue<HEEdgeWithNewPos , std::vector<HEEdgeWithNewPos >, GreaterHEEdge>().swap(edge_queue);
        for(size_t i = 0; i != edges.size(); ++i)
        {
            if(updated[i]) continue;
            updated[i] = true;
            if(IsValid(edges[i]->twin_edge))
            {
                // if(edges[i]->twin_edge->id >= updated.size() || edges[i]->twin_edge->id < 0)
                // std::cout<<edges[i]->twin_edge<<" "<<edges[i]->twin_edge->id<<" "<<updated.size()<<std::endl;
                updated[edges[i]->twin_edge->id] = true;
            }
            double e; 
            Point3 new_p;
            std::tie(new_p, e) = ComputeEdgeError(i);
            edges[i]->weight = e;
            edge_queue.push(HEEdgeWithNewPos(edges[i], new_p));
        }
    }
    void Decimator::ClusteringConfig()
    {
        auto &vertices = he.vertices;
        auto &faces = he.faces;
        vertex_to_faces.clear();
        vertex_to_faces.resize(vertices.size(), std::vector<HEFace *>());
        clustered_vertex_num.clear();
        clustered_vertex_num.resize(vertices.size(), 0);
        for(size_t i = 0; i != faces.size(); ++i)
        {
            if(IsValid(faces[i]))
            {   
                int vid1 = faces[i]->inc_edge->ori_vertex->id;
                int vid2 = faces[i]->inc_edge->des_vertex->id;
                int vid3 = faces[i]->inc_edge->next_edge->des_vertex->id;
                vertex_to_faces[vid1].push_back(faces[i]);
                vertex_to_faces[vid2].push_back(faces[i]);
                vertex_to_faces[vid3].push_back(faces[i]);
            }
        }
    }
    void Decimator::LoadTriangleMesh(const TriangleMesh &mesh)
    {
        he.Reset();
        he.FromTriangleMesh(mesh);
    }
    bool Decimator::Flipped(HEVertex *v, const Point3 &np, HEFace *f1, HEFace *f2)
    {
        if(v == nullptr|| he.vertices[v->id] != v ) 
        {
            return true;
        }
        auto &cor_faces = vertex_to_faces[v->id];
        
        for(size_t i = 0; i != cor_faces.size(); ++i)
        {
            //find other vertex
            if(!IsValid(cor_faces[i])) 
            {
                continue;
            }
            if(cor_faces[i] == f1) continue;
            if(cor_faces[i] == f2) continue;
            if(dirty[cor_faces[i]->id]) return true;
            auto tmp_edge = cor_faces[i]->inc_edge;
            while(tmp_edge->ori_vertex != v)
            {
                tmp_edge = tmp_edge->next_edge;
            }
            HEVertex *vb = tmp_edge->des_vertex;
            HEVertex *vc = tmp_edge->next_edge->des_vertex;
            if(v == vb || vb == vc || v == vc)
            {
                DeleteFace(cor_faces[i]);
                continue;
            }
            Vector3 v1 = vb->coor - np;
            v1.normalize();
            Vector3 v2 = vc->coor - np;
            v2.normalize();
            Vector3 fn = planes[cor_faces[i]->id].head<3>();

            if(v1.dot(v2) >= (1 - DRAGON_EPS)) 
            { 
                return true;
            }
            Vector3 nn = v1.cross(v2);
            nn.normalize();
            
            if(nn.dot(fn) < 0.2) 
            {
                return true;
            }
        }
        return false;
    }
    bool Decimator::Flipped(HEVertex *v1, HEVertex *v2)
    {
        auto &faces1 = vertex_to_faces[v1->id];
        auto &faces2 = vertex_to_faces[v2->id];
        std::set<int> connected_vertex1, connected_vertex2;
        int vid1 = v1->id;
        int vid2 = v2->id;
        for(size_t i = 0; i != faces1.size(); ++i)
        {
            if(IsValid(faces1[i]))
            {
                int vida = faces1[i]->inc_edge->ori_vertex->id;
                int vidb = faces1[i]->inc_edge->des_vertex->id;
                int vidc = faces1[i]->inc_edge->next_edge->des_vertex->id;
                if(vida != vid1 && vida != vid2)
                connected_vertex1.insert(vida);
                if(vidb != vid1 && vidb != vid2)
                connected_vertex1.insert(vidb);
                if(vidc != vid1 && vidc != vid2)
                connected_vertex1.insert(vidc);
            }
        }

        for(size_t i = 0; i != faces2.size(); ++i)
        {
            if(IsValid(faces2[i]))
            {
                int vida = faces2[i]->inc_edge->ori_vertex->id;
                int vidb = faces2[i]->inc_edge->des_vertex->id;
                int vidc = faces2[i]->inc_edge->next_edge->des_vertex->id;
                if(vida != vid1 && vida != vid2)
                connected_vertex2.insert(vida);
                if(vidb != vid1 && vidb != vid2)
                connected_vertex2.insert(vidb);
                if(vidc != vid1 && vidc != vid2)
                connected_vertex2.insert(vidc);                
            }
        }
        int intersection = 0;
        for(auto iter = connected_vertex2.begin(); iter != connected_vertex2.end(); ++iter)
        {
            if(connected_vertex1.find(*iter) != connected_vertex1.end())
                intersection++;
            if(intersection > 2) return true;
        }
        if(he.is_border[vid1] && he.is_border[vid2]&&intersection>1) return true; 
        // std::cout<<"intersection: "<<intersection<<std::endl;
        return false;

    }
    bool Decimator::Flipped(HEEdgeWithNewPos &ep)
    {
        HEFace *f1 = ep.ptr->parent_face;
        HEFace *f2 = nullptr;
        if(IsValid(ep.ptr->twin_edge)) f2 = ep.ptr->twin_edge->parent_face;
        return Flipped(ep.ptr->ori_vertex, ep.new_pos, f1, f2) 
            || Flipped(ep.ptr->des_vertex, ep.new_pos, f1, f2) || Flipped(ep.ptr->ori_vertex, ep.ptr->des_vertex);
    }
    void Decimator::DeleteFace(HEFace *f)
    {
        if(!f || f->id == -1) return;
        
        current_triangle_num--;
        
        f->inc_edge->next_edge->id = -1;
        f->inc_edge->id = -1;
        f->id = -1;
        f->inc_edge->pre_edge->id = -1;
    }
    void Decimator::DirtyFace(HEFace *f)
    {
        dirty[f->id] = true;   
    }
    void Decimator::DeleteEdge(HEEdge *e)
    {
        int vid1 = e->ori_vertex->id, vid2 = e->des_vertex->id;
        // int vid3 = -1, vid4 = -1;
        for(size_t i = 0; i != vertex_to_faces[vid1].size(); ++i)
        {
            if(vertex_to_faces[vid1][i]->id >= 0) 
            {
                DirtyFace(vertex_to_faces[vid1][i]);
            }
        }
        for(size_t i = 0; i != vertex_to_faces[vid2].size(); ++i)
        {
            if(vertex_to_faces[vid2][i]->id >= 0) 
            {
                DirtyFace(vertex_to_faces[vid2][i]);
            }
        }             
        e->des_vertex->id = e->ori_vertex->id;        
        e->des_vertex->coor = e->ori_vertex->coor;
        if(IsValid(e->next_edge->twin_edge))
        e->next_edge->twin_edge->twin_edge = e->pre_edge->twin_edge;
        if(IsValid(e->pre_edge->twin_edge))
        e->pre_edge->twin_edge->twin_edge = e->next_edge->twin_edge;
        DeleteFace(e->parent_face);
        // if(e->ori_vertex->inc_edge == e)
        if(IsValid(e->twin_edge)) 
        {
            if(IsValid(e->twin_edge->next_edge->twin_edge))
            e->twin_edge->next_edge->twin_edge->twin_edge = e->twin_edge->pre_edge->twin_edge;
            if(IsValid(e->twin_edge->pre_edge->twin_edge))
            e->twin_edge->pre_edge->twin_edge->twin_edge = e->twin_edge->next_edge->twin_edge;            
            DeleteFace(e->twin_edge->parent_face);
        }        
    }
    void Decimator::DeleteDegeneratedFace()
    {
        auto &faces = he.faces;
        for(size_t i = 0; i != faces.size(); ++i)
        {
            if(!IsValid( faces[i])) continue;
            int v1 = faces[i]->inc_edge->ori_vertex->id;
            int v2 = faces[i]->inc_edge->des_vertex->id;
            int v3 = faces[i]->inc_edge->next_edge->des_vertex->id;
            if(v1 == v2 || v1 == v3 || v2 == v3)
            {
                // std::cout<<"degenerated face"<<RESET<<std::endl; 
                DeleteFace(faces[i]);
            }
        }
    }
    std::shared_ptr<TriangleMesh> Decimator::Collapse(int target_num)
    {
        //acture collapse
        for(int iter = 0; iter != max_iteration; ++iter)
        {
            error_threshold = 0.000000001* std::pow(30 ,iter);
            std::cout<<"iter: "<<iter<<" error_threshold: "<<error_threshold<<" current_num: "<<current_triangle_num<<std::endl;
            while(!edge_queue.empty())
            {
                if(current_triangle_num <= target_num) break;
                // std::cout<<"eject"<<std::endl;
                auto top = edge_queue.top();
                edge_queue.pop();
                //std::cout<<"check flip"<<std::endl;
                if(top.ptr->weight > error_threshold) 
                { 
                    // std::cout<<top.ptr->weight<<std::endl;
                    break;
                }
                if(!IsValid(top.ptr)) continue;
                // check if dirty
                if(dirty[top.ptr->parent_face->id]) continue;
                // if(top.ptr->twin_edge && top.ptr->twin_edge->id != -1 && dirty[top.ptr->twin_edge->parent_face->id]) continue;
                if(he.is_border[top.ptr->ori_vertex->id] != he.is_border[top.ptr->des_vertex->id] ) continue;
                // check if flipped
                if(Flipped(top)) 
                {
                    continue;
                }
                // top.ptr->des_vertex->id = -1;
                top.ptr->ori_vertex->coor = top.new_pos;
                // delete the face
                //std::cout<<"delete edge"<<std::endl;
                DeleteEdge(top.ptr);
            }
            // std::cout<<"update halfedge"<<std::endl;
            // DeleteDegeneratedFace();
            he.Update();
            if(current_triangle_num <= target_num) break;
            // std::cout<<iter<<" "<<current_triangle_num <<" "<<target_num <<std::endl;
            QuadricConfig();
            // break;
        }
        // he.CheckBorder();
        // for(size_t i = 0; i !=he.is_border.size(); ++i)
        // {
        //     if(he.is_border[i])
        //     he.vertices[i]->color = Point3(1, 1, 0);
        // }
        TriangleMesh mesh;
        he.ToTriangleMesh(mesh);
        return std::make_shared<TriangleMesh>(mesh);
    }
    // Point3i GetGridIndex(const geometry::Point3 &points, double grid_len)
    // {
    //     return Point3i(std::floor(points(0)/grid_len), std::floor(points(1)/grid_len),std::floor(points(2)/grid_len));
    // }    
    std::shared_ptr<TriangleMesh> Decimator::Cluster(double grid_len)
    {
        auto &vertices = he.vertices;
        // auto &faces = he.faces;        
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            Point3i voxel_id = geometry::GetGridIndex(vertices[i]->coor, grid_len);
            if(grid_map.find(voxel_id) == grid_map.end())
            {
                grid_map[voxel_id] = vertices[i];
            }
            else
            {
                grid_map[voxel_id]->coor += vertices[i]->coor;
                vertices[i]->id = grid_map[voxel_id]->id;   
            }
            clustered_vertex_num[grid_map[voxel_id]->id] ++;
        }
        for(auto iter = grid_map.begin(); iter != grid_map.end(); ++iter)
        {
            iter->second->coor = iter->second->coor / clustered_vertex_num[iter->second->id];
        }
        he.Update();
        DeleteDegeneratedFace();
        he.Update();
        TriangleMesh mesh; 
        he.ToTriangleMesh(mesh);
        return std::make_shared<TriangleMesh>(mesh);
    }
    std::shared_ptr<TriangleMesh> QuadricDecimation(const TriangleMesh &mesh, size_t target_num)
    {
        Decimator decimator;
        decimator.LoadTriangleMesh(mesh);
        decimator.QuadricConfig();
        auto res = decimator.Collapse(target_num);
        std::cout<<GREEN<<"Finish decimation, "<<mesh.triangles.size() - res->triangles.size()<<" triangles are decimated."<<RESET<<std::endl;
        return res;
    }
    std::shared_ptr<TriangleMesh> ClusteringDecimation(const TriangleMesh &mesh, double voxel_len)
    {
        Decimator decimator;
        decimator.LoadTriangleMesh(mesh);
        decimator.ClusteringConfig();
        auto res = decimator.Cluster(voxel_len);
        std::cout<<GREEN<<"Finish decimation, "<<mesh.triangles.size() - res->triangles.size()<<" triangles are decimated."<<RESET<<std::endl;
        return res;        
    }
    
}
}
}