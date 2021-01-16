#include "Poisson.h"
#include "Geometry/Structure/Octree.h"
#include "Geometry/Structure/KDTree.h"
namespace dragon
{
namespace reconstruction
{
    double ThirdBoxFilter(double x)
    {
        if(x <= -1.5)
        return 0;
        else if(x > - 1.5 && x <= -0.5)
        return 0.5 * x * x + 1.5 * x + 1.125;
        else if(x > -0.5 && x <= 0.5)
        return 0.75 - x * x;
        else if(x > 0.5 && x <= 1.5)
        return 0.5 * x * x - 1.5 * x + 1.125;
        else
        return 0;
    }
    double DeThirdBoxFilter(double x)
    {
        if(x <= -1.5)
        return 0;
        else if(x > - 1.5 && x <= -0.5)
        return x + 1.5;
        else if(x > -0.5 && x <= 0.5)
        return - 2 * x;
        else if(x > 0.5 && x <= 1.5)
        return x - 1.5;
        else
        return 0;
    }
    double De2ThirdBoxFilter(double x)
    {
        if(x <= -1.5)
        return 0;
        else if(x > - 1.5 && x <= -0.5)
        return 1;
        else if(x > -0.5 && x <= 0.5)
        return - 2;
        else if(x > 0.5 && x <= 1.5)
        return 1;
        else
        return 0;
    }
    double  ThirdBoxFilter3D(const geometry::Point3 &p)
    {
        return ThirdBoxFilter(p(0)) * ThirdBoxFilter(p(1)) * ThirdBoxFilter(p(2));
    }
    geometry::Point3 DeThirdBoxFilter3D(const geometry::Point3 &p)
    {
        return geometry::Point3(DeThirdBoxFilter(p(0)) * ThirdBoxFilter(p(1)) * ThirdBoxFilter(p(2)), 
            ThirdBoxFilter(p(0)) * DeThirdBoxFilter(p(1)) * ThirdBoxFilter(p(2)), ThirdBoxFilter(p(0)) * ThirdBoxFilter(p(1)) * DeThirdBoxFilter(p(2)) );
    }
    geometry::Point3 LaplacianThirdBoxFilter3D(const geometry::Point3 &p)
    {
        return geometry::Point3(De2ThirdBoxFilter(p(0)) * ThirdBoxFilter(p(1)) * ThirdBoxFilter(p(2)), 
            ThirdBoxFilter(p(0)) * De2ThirdBoxFilter(p(1)) * ThirdBoxFilter(p(2)), ThirdBoxFilter(p(0)) * ThirdBoxFilter(p(1)) * De2ThirdBoxFilter(p(2)) );
    }
    std::shared_ptr<geometry::TriangleMesh> Poisson(const geometry::PointCloud &pcd, int max_depth)
    {
        geometry::Octree octree(max_depth);
        // octree.UniformSplit();
        octree.BuildTree(pcd);
        
        auto points = pcd.points;
        auto normals = pcd.normals;
        int computed_node = 0;
        geometry::VectorX known_indicator;

        for(int iter = 1; iter <= max_depth; ++iter)
        {
            std::vector<geometry::OctreeNode *> &nodes = octree.all_nodes[iter];
            // compute the basis function, box filter
            // 3-th box filter
            geometry::Point3List d_points(nodes.size(), geometry::Point3::Zero());
            geometry::Point3List d_normals(nodes.size(), geometry::Point3::Zero());
            for(size_t i = 0; i != nodes.size(); ++i)
            {
                auto &pidlist = nodes[i]->pidlist;
                for(size_t j = 0; j != pidlist.size(); ++j)
                {
                    d_points[i] += points[pidlist[j]];
                    d_normals[i] += normals[pidlist[j]];
                }
                if(pidlist.size() > 0)
                {
                    d_points[i] /= pidlist.size();
                    d_normals[i] /= pidlist.size();
                }
            }
            // the weight of basis function
            // compute each vector field based on the basis function
            for(size_t i = 0; i != d_points.size(); ++i)
            {
                // splat the sample points
                if(d_points[i].norm() == 0) continue;
                double half_width = nodes[i]->width / 2;
                std::vector<geometry::OctreeNode *> neighbor_nodes;
                for(int j = 0; j != 8; ++j)
                {
                    geometry::Point3 tmp_p = d_points[i];
                    if(j & 1)
                    tmp_p(0) += half_width;
                    else tmp_p(0) -= half_width;

                    if((j>>1) & 1)
                    tmp_p(1) += half_width;
                    else tmp_p(1) -= half_width;

                    if((j>>2) & 1)
                    tmp_p(2) += half_width;
                    else tmp_p(2) -= half_width;

                    octree.LocateAndSplit(tmp_p, iter);
                    // std::cout<<n<<std::endl;
                    // if(n == nullptr)
                    // {
                    //     std::cout<<"what happened? "<<iter<<std::endl;
                    // }
                }
            } 
            
        }
        octree.SetNodeIDAndGetAllNodes();
        for(int iter = 1; iter <= max_depth; ++iter)
        {
            std::vector<geometry::OctreeNode *> &nodes = octree.all_nodes[iter];
            std::cout<<"nodes: "<<nodes.size()<<std::endl;
            // compute the basis function, box filter
            // 3-th box filter
            geometry::Point3List d_points(nodes.size(), geometry::Point3::Zero());
            geometry::Point3List d_normals(nodes.size(), geometry::Point3::Zero());
            for(size_t i = 0; i != nodes.size(); ++i)
            {
                auto &pidlist = nodes[i]->pidlist;
                for(size_t j = 0; j != pidlist.size(); ++j)
                {
                    d_points[i] += points[pidlist[j]];
                    d_normals[i] += normals[pidlist[j]];
                }
                if(pidlist.size() > 0)
                {
                    d_points[i] /= pidlist.size();
                    d_normals[i] /= pidlist.size();
                }
            }
            Eigen::SparseMatrix<geometry::scalar> basis_funcs(nodes.size() , nodes.size());
            std::vector<Eigen::Triplet<geometry::scalar>> coefficients;
            for(size_t i = 0; i != nodes.size(); ++i)
            {
                for(size_t j = 0; j != nodes.size(); ++j)
                {
                    double fvalue = ThirdBoxFilter3D((nodes[i]->center - nodes[j]->center) / nodes[j]->width) / std::pow(nodes[j]->width, 3);
                    if(fvalue != 0)
                    {
                        coefficients.push_back(Eigen::Triplet<geometry::scalar>(j, i, fvalue));
                    }
                }

            }
            basis_funcs.setFromTriplets(coefficients.begin(), coefficients.end());

            geometry::MatrixX weighted_normal(nodes.size(), 3);
            weighted_normal.setZero();
            // the weight of basis function
            // compute each vector field based on the basis function
            // if(iter > 1)
            // for(size_t i = 0; i != d_points.size(); ++i)
            // {
            //     weighted_normal.block<1, 3>(i, 0) += nodes[i]->normal;
            // }
            for(size_t i = 0; i != d_points.size(); ++i)
            {
                // splat the sample points
                if(d_points[i].norm() == 0) 
                {
                    continue;
                }
                double half_width = nodes[i]->width / 2;
                std::vector<geometry::OctreeNode *> neighbor_nodes;
                for(int j = 0; j != 8; ++j)
                {
                    geometry::Point3 tmp_p = d_points[i];
                    if(j & 1)
                    tmp_p(0) += half_width;
                    else tmp_p(0) -= half_width;

                    if((j>>1) & 1)
                    tmp_p(1) += half_width;
                    else tmp_p(1) -= half_width;

                    if((j>>2) & 1)
                    tmp_p(2) += half_width;
                    else tmp_p(2) -= half_width;

                    auto n = octree.Locate(tmp_p, iter);
                    // std::cout<<n<<std::endl;
                    if(n != nullptr)
                    {
                        // std::cout<<n->id<<" "<<n->depth<<" "<<iter<<std::endl;
                        neighbor_nodes.push_back(n);
                    }
                    // else
                    // {
                    //     std::cout<<"Big problem"<<std::endl;
                    // }
                }
                for(size_t nid = 0; nid != neighbor_nodes.size(); ++nid)
                {
                    // the weight is based on the distance: weight = 1 - d / (3^(0.5))
                    double weight = 1 - (neighbor_nodes[nid]->center - d_points[i]).norm() / (1.732051 * octree.head.width);
                    // std::cout<<neighbor_nodes[nid]->id<<std::endl;
                    weighted_normal.block<1, 3>(neighbor_nodes[nid]->id, 0) -= weight * d_normals[i].transpose();
                }
            } 

            // compute the normal of nodes
            // double max_length = 0;
#if 0
            for(size_t i = 0; i != nodes.size(); ++i)
            {
                nodes[i]->normal = weighted_normal.block<1, 3>(i, 0);
                // if(!nodes[i]->is_leaf)
                // {
                //     for(int nid = 0; nid != 8; ++nid)
                //     nodes[i]->nodes[nid].normal = nodes[i]->normal;
                // }
            }
#endif

            // compute the divergence of normal
            Eigen::SparseMatrix<geometry::scalar> de_basis_function_x(nodes.size(), nodes.size());
            Eigen::SparseMatrix<geometry::scalar> de_basis_function_y(nodes.size(), nodes.size());
            Eigen::SparseMatrix<geometry::scalar> de_basis_function_z(nodes.size(), nodes.size());

            std::vector<Eigen::Triplet<geometry::scalar>> coefficients_x, coefficients_y, coefficients_z;
            
            for(size_t i = 0; i != nodes.size(); ++i)
            {
                for(size_t j = 0; j != nodes.size(); ++j)
                {
                    geometry::Point3 de_value = DeThirdBoxFilter3D((nodes[i]->center - nodes[j]->center) / nodes[j]->width);
                    de_value /= std::pow(nodes[j]->width, 4);
                    if(de_value(0) != 0.0)
                    {
                        coefficients_x.push_back(Eigen::Triplet<geometry::scalar>(i, j, de_value(0)));
                    }
                    if(de_value(1) != 0.0)
                    {
                        coefficients_y.push_back(Eigen::Triplet<geometry::scalar>(i, j, de_value(1)));
                    }
                    if(de_value(2) != 0.0)
                    {
                        coefficients_z.push_back(Eigen::Triplet<geometry::scalar>(i, j, de_value(2)));
                    }
                }
            }
            de_basis_function_x.setFromTriplets(coefficients_x.begin(), coefficients_x.end());
            de_basis_function_y.setFromTriplets(coefficients_y.begin(), coefficients_y.end());
            de_basis_function_z.setFromTriplets(coefficients_z.begin(), coefficients_z.end());

            geometry::MatrixX div_v(nodes.size(), 3);
            div_v.block(0, 0, nodes.size(), 1) = de_basis_function_x * weighted_normal.block(0, 0, nodes.size(), 1);
            div_v.block(0, 1, nodes.size(), 1) = de_basis_function_y * weighted_normal.block(0, 1, nodes.size(), 1);
            div_v.block(0, 2, nodes.size(), 1) = de_basis_function_z * weighted_normal.block(0, 2, nodes.size(), 1);
            // project the div_v into function space
            geometry::VectorX projected_div_v(nodes.size());
            projected_div_v.setZero();
            projected_div_v.block(0, 0,  nodes.size(), 1) += basis_funcs * div_v.block(0, 0, nodes.size(), 1);
            projected_div_v.block(0, 0,  nodes.size(), 1) += basis_funcs * div_v.block(0, 1, nodes.size(), 1);
            projected_div_v.block(0, 0,  nodes.size(), 1) += basis_funcs * div_v.block(0, 2, nodes.size(), 1);


            // compute the laplacian of basis function
            coefficients_x.clear();
            coefficients_y.clear();
            coefficients_z.clear();

            Eigen::SparseMatrix<geometry::scalar> lap_basis_function_x(nodes.size(), nodes.size() );
            Eigen::SparseMatrix<geometry::scalar> lap_basis_function_y(nodes.size(), nodes.size() );
            Eigen::SparseMatrix<geometry::scalar> lap_basis_function_z(nodes.size(), nodes.size() );

            for(size_t i = 0; i != nodes.size(); ++i)
            {
                for(size_t j = 0; j != nodes.size(); ++j)
                {
                    geometry::Point3 dede_value = LaplacianThirdBoxFilter3D((nodes[i]->center - nodes[j]->center) / nodes[j]->width);
                    dede_value /= std::pow(nodes[j]->width, 5);
                    if(dede_value(0) != 0.0)
                    {
                        coefficients_x.push_back(Eigen::Triplet<geometry::scalar>(i, j , dede_value(0)));
                    }
                    if(dede_value(1) != 0.0)
                    {
                        coefficients_y.push_back(Eigen::Triplet<geometry::scalar>(i, j , dede_value(1)));
                    }
                    if(dede_value(2) != 0.0)
                    {
                        coefficients_z.push_back(Eigen::Triplet<geometry::scalar>(i, j , dede_value(2)));
                    }
                }

            }
            lap_basis_function_x.setFromTriplets(coefficients_x.begin(), coefficients_x.end());
            lap_basis_function_y.setFromTriplets(coefficients_y.begin(), coefficients_y.end());
            lap_basis_function_z.setFromTriplets(coefficients_z.begin(), coefficients_z.end());

            // project the laplacian operator onto function space
            // construct the L matrix
            Eigen::SparseMatrix<geometry::scalar> L =(basis_funcs * lap_basis_function_x ).pruned() 
                + (basis_funcs * lap_basis_function_y ).pruned() + (basis_funcs * lap_basis_function_z ).pruned();

            // Eigen::SparseMatrix<geometry::scalar> L_t = L.transpose();
            // solve poisson equation
#if 1
            Eigen::SparseLU<Eigen::SparseMatrix<geometry::scalar>, Eigen::COLAMDOrdering<int>> slu_solver;


            // std::cout<<L<<std::endl;
            geometry::VectorX indicator(nodes.size());
            slu_solver.compute(L);
            indicator = slu_solver.solve(projected_div_v);
          
            for(size_t i = 0; i != nodes.size(); ++i)
            {
                // std::cout<<nodes[i]->depth<<" "<<indicator(i)<<std::endl;
                nodes[i]->weight = indicator(i);
            }
#endif
            computed_node += nodes.size();
        }
        octree.GetPointCloud()->WriteToPLY("octree_pcd.ply");
        // std::cout<<indicator<<std::endl;
        double iso_value = 0.0;
        double max_iso_value = std::numeric_limits<double>::lowest();
        double min_iso_value = std::numeric_limits<double>::max();
        auto &all_nodes = octree.all_nodes;
        for(size_t i = 0; i != points.size(); ++i)
        {
            for(size_t j = 1; j != all_nodes.size(); ++j)
            {
                for(size_t k = 0; k != all_nodes[j].size(); ++k)
                {
                    auto current_node = all_nodes[j][k];
                    double fvalue = ThirdBoxFilter3D((points[i] - current_node->center) / current_node->width) / std::pow(current_node->width, 3);
                    if(fvalue != 0)
                    {
                        // std::cout<<fvalue<<std::endl;
                        double tmp_value = fvalue * current_node->weight;
                        iso_value += tmp_value;
                        if(tmp_value > max_iso_value)
                        max_iso_value = tmp_value;
                        if(tmp_value < min_iso_value)
                        min_iso_value = tmp_value; 
                    }
                }
            }      
        }
        iso_value /= points.size();
        std::cout<<iso_value<<std::endl;
        CubeHandler cube_handler;
        std::cout<<std::max((max_iso_value - iso_value) / iso_value, (iso_value - min_iso_value) / iso_value + 0.2)<<std::endl; 
        cube_handler.SetVoxelResolution(octree.head.width / std::pow(2, max_depth) / 2.75);
        cube_handler.SetTruncation(std::max((max_iso_value - iso_value) / iso_value, (iso_value - min_iso_value) / iso_value));
        std::function<double (geometry::Point3)> get_sdf = [&](geometry::Point3 p)-> double {
        double ivalue = 0;
        for(size_t j =  1; j != all_nodes.size(); ++j)
        {
            for(size_t k = 0; k != all_nodes[j].size(); ++k)
            {
                auto current_node = all_nodes[j][k];
                double fvalue = ThirdBoxFilter3D((p - current_node->center) / current_node->width) / std::pow(current_node->width, 3);
                if(fvalue != 0)
                {
                    // std::cout<<fvalue<<std::endl;
                    ivalue += fvalue * current_node->weight;
                }
            }
        }   
        // std::cout<<ivalue - iso_value<<std::endl;
        return ivalue / iso_value - 1;
        };        
        cube_handler.IntegratePoints(points, get_sdf);
        std::cout<<BLUE<<"[INFO]::[RBF]::Finish points integration, extracting mesh ..."<<RESET<<std::endl;
        geometry::TriangleMesh mesh;
        cube_handler.ExtractTriangleMesh(mesh);
        if(pcd.HasColors())
        {
            geometry::KDTree<3> kdtree;
            kdtree.BuildTree(points);
            for(size_t i = 0; i != mesh.points.size(); ++i)
            {
                std::vector<int> indices; 
                std::vector<float> dists; 
                kdtree.KnnSearch(mesh.points[i], indices, dists,
                    1,  geometry::SearchParameter(32));     
                if(indices.size())
                mesh.colors[i] = pcd.colors[indices[0]];           
            }
        }       
        else
        {
            for(size_t i = 0; i != mesh.points.size(); ++i)
            mesh.colors[i] = geometry::Point3(1, 1, 1);
        }
        return std::make_shared<geometry::TriangleMesh>(mesh); 
    }
}
}