#include "Curvature.h"
namespace dragon
{
namespace geometry
{
    // compute mean curvature
    void ComputeMeanCurvature(const HalfEdge &he, Point3List &mean_curvature_vectors)
    {
        mean_curvature_vectors.clear();
        auto &vertices = he.vertices;
        auto &faces = he.faces;
        //auto &edges = he.edges;
        auto &is_border = he.is_border;
        Point3List circum_centers;
        std::vector<int> triangle_type; 
        if(is_border.size() == 0)
        {
            std::cout<<RED<< "[ERROR]::[MeanCurvature]::You need to call CheckBorder firstly."<<RESET<<std::endl;
            return;
        }
        for(size_t i = 0; i != faces.size(); ++i)
        {
            Point3 &a = faces[i].inc_edge->ori_vertex->coor;
            Point3 &b = faces[i].inc_edge->des_vertex->coor;
            Point3 &c = faces[i].inc_edge->next_edge->des_vertex->coor;
            // compute the circum center and triangle type
            // we only want acute angle
            triangle_type.push_back(TriangleType(a, b, c));
            circum_centers.push_back(CircumCenter(a, b, c));
        }
        
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            //compute mean Curvature
            // first step, find all 1-neighbor
            geometry::Vector3 mean_curvature_vector(0.0, 0.0, 0.0);
            double sum_area = 0.0;

            if(is_border[i])
            {
                mean_curvature_vectors.push_back(mean_curvature_vector);
                continue;
            }
            auto start_edge = vertices[i].inc_edge;
            auto current_edge = start_edge->twin_edge->next_edge;
            std::vector<size_t> first_neighbors;
            std::vector<size_t> connected_faces;

            first_neighbors.push_back(start_edge->des_vertex->id);
            connected_faces.push_back(start_edge->parent_face->id);
            while(current_edge != start_edge)
            {
                first_neighbors.push_back(current_edge->des_vertex->id);
                connected_faces.push_back(current_edge->parent_face->id);
                current_edge = current_edge->twin_edge->next_edge;
            }

            for(size_t id = 0; id != first_neighbors.size(); ++id)
            {
                int last_id = id - 1;
                if(last_id < 0) last_id = first_neighbors.size() - 1;
                int next_id = (id + 1) % first_neighbors.size();   
                int last_face_id = connected_faces[last_id], current_face_id = connected_faces[id];
                
                Point3 chosen_p_0, chosen_p_1;      
                const Point3 &v_last = vertices[first_neighbors[last_id]].coor;
                const Point3 &v_next = vertices[first_neighbors[next_id]].coor;
                const Point3 &v_current = vertices[first_neighbors[id]].coor;
                const Point3 &v_ = vertices[i].coor;

                Point3 edge_center =  (v_current + v_)/2;

                //acute type
                if(triangle_type[last_face_id] == 0)
                {
                    chosen_p_0 = circum_centers[last_face_id];
                }
                else
                {
                    chosen_p_0 = (v_last + v_current) / 2;
                }

                if(triangle_type[current_face_id] == 0)
                {
                    chosen_p_1 = circum_centers[current_face_id];
                }
                else
                {
                    chosen_p_1 = (v_next + v_current) / 2;
                }
                //compute area
                sum_area += ComputeTriangleArea(v_, edge_center, chosen_p_0);
                sum_area += ComputeTriangleArea(v_, edge_center, chosen_p_1);
                // sum_area += ComputeTriangleArea(v_, v_current, v_last);
                // sum_area += ComputeTriangleArea(v_, v_current, v_next);
                // compute angles
                double angle1 = AngleOfVector(v_ - v_last, v_current - v_last);
                double angle2 = AngleOfVector(v_ - v_next, v_current - v_next);
                mean_curvature_vector += (cot(angle1) + cot(angle2)) * (v_current - v_);
            }

            mean_curvature_vectors.push_back( mean_curvature_vector / sum_area / 2);
        }
    }
    void ComputeGaussCurvature(const HalfEdge &he, std::vector<double> &gauss_curvatures)
    {
        gauss_curvatures.clear();
        auto &vertices = he.vertices;
        auto &faces = he.faces;
        //auto &edges = he.edges;
        auto &is_border = he.is_border;
        Point3List circum_centers;
        std::vector<int> triangle_type; 
        if(is_border.size() == 0)
        {
            std::cout<<RED<< "[ERROR]::[GaussCurvature]::You need to call CheckBorder firstly."<<RESET<<std::endl;
            return;
        }
        for(size_t i = 0; i != faces.size(); ++i)
        {
            Point3 &a = faces[i].inc_edge->ori_vertex->coor;
            Point3 &b = faces[i].inc_edge->des_vertex->coor;
            Point3 &c = faces[i].inc_edge->next_edge->des_vertex->coor;
            // compute the circum center and triangle type
            // we only want acute angle
            triangle_type.push_back(TriangleType(a, b, c));
            circum_centers.push_back(CircumCenter(a, b, c));
        }
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            if(is_border[i])
            {
                gauss_curvatures.push_back(0);
                continue;
            }
            auto start_edge = vertices[i].inc_edge;
            auto current_edge = start_edge->twin_edge->next_edge;
            std::vector<size_t> first_neighbors;
            std::vector<size_t> connected_faces;

            first_neighbors.push_back(start_edge->des_vertex->id);
            connected_faces.push_back(start_edge->parent_face->id);
            while(current_edge != start_edge)
            {
                first_neighbors.push_back(current_edge->des_vertex->id);
                connected_faces.push_back(current_edge->parent_face->id);
                current_edge = current_edge->twin_edge->next_edge;
            }
            double sum_angle = 0;
            double sum_area = 0;
            for(size_t id = 0; id != first_neighbors.size(); ++id)
            {

                int last_id = id - 1;
                if(last_id < 0) last_id = first_neighbors.size() - 1;
                int next_id = (id + 1) % first_neighbors.size();   
                int last_face_id = connected_faces[last_id], current_face_id = connected_faces[id];
                
                Point3 chosen_p_0, chosen_p_1;      
                const Point3 &v_last = vertices[first_neighbors[last_id]].coor;
                const Point3 &v_next = vertices[first_neighbors[next_id]].coor;
                const Point3 &v_current = vertices[first_neighbors[id]].coor;
                const Point3 &v_ = vertices[i].coor;

                Point3 edge_center =  (v_current + v_)/2;

                //acute type
                if(triangle_type[last_face_id] == 0)
                {
                    chosen_p_0 = circum_centers[last_face_id];
                }
                else
                {
                    chosen_p_0 = (v_last + v_current) / 2;
                }

                if(triangle_type[current_face_id] == 0)
                {
                    chosen_p_1 = circum_centers[current_face_id];
                }
                else
                {
                    chosen_p_1 = (v_next + v_current) / 2;
                }
                //compute area
                sum_area += ComputeTriangleArea(v_, edge_center, chosen_p_0);
                sum_area += ComputeTriangleArea(v_, edge_center, chosen_p_1);
                // sum_area += ComputeTriangleArea(v_, v_current, v_last);
                // sum_area += ComputeTriangleArea(v_, v_current, v_next);

                // compute angles
                // std::cout<<"angle: "<<AngleOfVector(v_current - v_, v_next - v_) / M_PI * 180<<std::endl;
                sum_angle += AngleOfVector(v_current - v_, v_next - v_);
            }
            // std::cout<<"first neighbor: "<<first_neighbors.size()<<std::endl;
            // std::cout<<"sum_angle: "<<sum_angle<<" "<<2 * M_PI <<std::endl;
            // std::cout<<"sum_area: "<<sum_area<<std::endl;
            gauss_curvatures.push_back( (2 * M_PI -  sum_angle) / sum_area);
        }
    }
}
}