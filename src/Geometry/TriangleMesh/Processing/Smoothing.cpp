#include "Smoothing.h"
#include "Curvature.h"
namespace dragon
{
namespace geometry
{
namespace mesh
{
    // Compute new position for each vertex
    void NaiveLaplacianSmooting(const HalfEdge &he, Point3List &new_positions)
    {
        new_positions.clear();
        auto &vertices = he.vertices;
        //auto &edges = he.edges;
        auto &is_border = he.is_border;
        if(is_border.size() == 0)
        {
            std::cout<<RED<< "[ERROR]::[LaplacianSmooting]::You need to call CheckBorder firstly."<<RESET<<std::endl;
            return;
        }
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            //compute mean Curvature
            // first step, find all 1-neighbor
            geometry::Vector3 new_position(0.0, 0.0, 0.0);
            if(is_border[i])
            {
                new_positions.push_back(new_position);
                continue;
            }
            auto start_edge = vertices[i].inc_edge;
            auto current_edge = start_edge->twin_edge->next_edge;
            std::vector<size_t> first_neighbors;

            first_neighbors.push_back(start_edge->des_vertex->id);
            while(current_edge != start_edge)
            {
                first_neighbors.push_back(current_edge->des_vertex->id);
                current_edge = current_edge->twin_edge->next_edge;
            }

            for(size_t id = 0; id != first_neighbors.size(); ++id)
            {
                new_position += vertices[first_neighbors[id]].coor;
            }

            new_positions.push_back( new_position / first_neighbors.size());
        }
    }
    std::shared_ptr<TriangleMesh> NaiveLaplacianSmooting(const TriangleMesh &mesh, double lambda, int max_iteration)
    {
        HalfEdge he; 
        he.FromTriangleMesh(mesh);
        // std::cout<<"he: "<<he.faces.size()<<"/"<<he.edges.size()<<"/"<<he.vertices.size()<<std::endl;
        he.CheckBorder();
        Point3List new_positions;
        int iter = 0;
        auto &vertices = he.vertices;
        auto is_border = he.is_border;

        while(iter < max_iteration)
        {
            NaiveLaplacianSmooting(he, new_positions);
            for(size_t i = 0; i != vertices.size(); ++i)
            {
                if(is_border[i])
                continue;
                // update the vertices
                vertices[i].coor += (new_positions[i] - vertices[i].coor) * lambda; 
                // if(new_positions[i].norm() > 1)
                // std::cout<<new_positions[i].transpose()<<std::endl;
            }
            iter ++;
        }
        TriangleMesh result;
        he.ToTriangleMesh(result);
        return std::make_shared<TriangleMesh>(result);
    }
    std::shared_ptr<TriangleMesh> LocalLaplacianSmooting(const TriangleMesh &mesh, double lambda, int max_iteration)
    {
        HalfEdge he; 
        he.FromTriangleMesh(mesh);
        // std::cout<<"he: "<<he.faces.size()<<"/"<<he.edges.size()<<"/"<<he.vertices.size()<<std::endl;
        he.CheckBorder();
        ComputeCotanWeight(he);
        Point3List update_vectors;
        int iter = 0;
        auto &vertices = he.vertices;
        auto is_border = he.is_border;

        while(iter < max_iteration)
        {
            for(size_t i = 0; i != vertices.size(); ++i)
            {
                if(is_border[i])
                continue;
                auto start_edge = vertices[i].inc_edge;
                auto current_edge = start_edge->twin_edge->next_edge;
                double sum_weight = 0.0;
                geometry::Vector3 update_vector = geometry::Vector3::Zero();
                sum_weight += start_edge->weight;
                update_vector += start_edge->weight * (start_edge->ori_vertex->coor - start_edge->des_vertex->coor);
                while(current_edge != start_edge)
                {
                    sum_weight += current_edge->weight;
                    update_vector += current_edge->weight * (current_edge->ori_vertex->coor - current_edge->des_vertex->coor);
                    current_edge = current_edge->twin_edge->next_edge;
                }
                // std::cout<<update_vector.transpose()<<std::endl;
                vertices[i].coor -= lambda * update_vector / sum_weight;
            }
            iter ++;
        }
        TriangleMesh result;
        he.ToTriangleMesh(result);
        return std::make_shared<TriangleMesh>(result);
    }
    std::shared_ptr<TriangleMesh> GlobalLaplacianSmooting(const TriangleMesh &mesh, double lambda)
    {
        HalfEdge he; 
        he.FromTriangleMesh(mesh);
        // std::cout<<"he: "<<he.faces.size()<<"/"<<he.edges.size()<<"/"<<he.vertices.size()<<std::endl;
        he.CheckBorder();
        ComputeCotanWeight(he);
        auto &vertices = he.vertices;
        //auto &edges = he.edges;
        auto &is_border = he.is_border;
        Eigen::SparseMatrix<geometry::scalar> laplace_matrix(3 * he.vertices.size(), 3 * he.vertices.size());
        geometry::MatrixX sigma(3 * vertices.size(), 1), new_vertices(3 * vertices.size(), 1);
        sigma.setZero();
        std::vector<Eigen::Triplet<geometry::scalar>> coefficients;
        for(size_t i = 0; i != vertices.size(); ++i)
        {
            geometry::Matrix3 tmp_l = geometry::Matrix3::Identity();
            geometry::Vector3 tmp_sigma = geometry::Vector3::Zero();
            AddToCoefficientTriplet(coefficients, i *3, i*3, tmp_l);
            if(is_border[i])
            {
                tmp_sigma = vertices[i].coor;
                sigma.block<3, 1>(3*i, 0) += tmp_sigma;
                continue;
            }
            //set L matrix
            auto start_edge = vertices[i].inc_edge;
            auto current_edge = start_edge->twin_edge->next_edge;
            std::vector<size_t> first_neighbors;
            std::vector<double> cotan_weights;
            double sum_weight = 0.0;
            first_neighbors.push_back(start_edge->des_vertex->id);
            cotan_weights.push_back(start_edge->weight);
            sum_weight += start_edge->weight;
            while(current_edge != start_edge)
            {
                cotan_weights.push_back(current_edge->weight);
                first_neighbors.push_back(current_edge->des_vertex->id);
                sum_weight += current_edge->weight;
                current_edge = current_edge->twin_edge->next_edge;
            }
            //first_neighbors

            for(size_t id = 0; id != first_neighbors.size(); ++id)
            {
                tmp_l = - geometry::Matrix3::Identity() * cotan_weights[id] / sum_weight;
                AddToCoefficientTriplet(coefficients, i * 3, first_neighbors[id] * 3, tmp_l);
            }
            //set sigma vector
            tmp_sigma = vertices[i].coor * lambda;
            sigma.block<3, 1>(3*i, 0) += tmp_sigma;
        }
        laplace_matrix.setZero();
        laplace_matrix.setFromTriplets(coefficients.begin(), coefficients.end());
        //std::cout<<laplace_matrix<<std::endl;
        Eigen::SparseLU<Eigen::SparseMatrix<geometry::scalar>, Eigen::COLAMDOrdering<int>> slu_solver;
        slu_solver.compute(laplace_matrix);
        new_vertices = slu_solver.solve(sigma);
        //Restore vertices
        for(size_t i = 0; i != vertices.size(); ++i)
        vertices[i].coor = new_vertices.block<3, 1>(i * 3, 0);
        TriangleMesh result;
        he.ToTriangleMesh(result);
        return std::make_shared<TriangleMesh>(result);
    }
}
}
}