#include "Poisson.h"
#include "Geometry/Structure/Octree.h"
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
        octree.BuildTree(pcd);
        auto points = pcd.points;
        auto normals = pcd.normals;
        
        std::vector<geometry::OctreeNode *> &leaves = octree.all_leaves;
        std::cout<<"Leaves: "<<leaves.size()<<std::endl;
        // compute the basis function, box filter
        // 3-th box filter
        Eigen::SparseMatrix<geometry::scalar> basis_funcs(leaves.size(), leaves.size());
        std::vector<Eigen::Triplet<geometry::scalar>> coefficients;
        for(size_t i = 0; i != leaves.size(); ++i)
        {
            for(size_t j = 0; j != leaves.size(); ++j)
            {
                double fvalue = ThirdBoxFilter3D((leaves[i]->center - leaves[j]->center) / leaves[j]->width) / std::pow(leaves[j]->width, 3);
                if(fvalue != 0)
                coefficients.push_back(Eigen::Triplet<geometry::scalar>(i, j, fvalue));
            }
        }
        basis_funcs.setFromTriplets(coefficients.begin(), coefficients.end());
        geometry::Point3List weighted_normal;
        // the weight of basis function
        weighted_normal.resize(leaves.size(), geometry::Point3::Zero());
        // compute each vector field based on the basis function
        for(size_t i = 0; i != points.size(); ++i)
        {
            // splat the sample points
            geometry::OctreeNode * parent = octree.point_to_leaf[i]->parent;
            if(parent != nullptr)
            {
                std::vector<geometry::OctreeNode *> tmp_leaves;
                parent->GetAllLeaves(tmp_leaves);
                for(size_t nid = 0; nid != tmp_leaves.size(); ++nid)
                {
                    // the weight is based on the distance: weight = 1 - d / (3^(0.5))
                    double weight = 1 - (tmp_leaves[nid]->center - points[i]).norm() / (1.732051 * octree.head.width);
                    weighted_normal[tmp_leaves[nid]->id] += weight * normals[i];
                }
            }
        } 
        
        // compute the normal of leaves
        double max_length = 0;
        for(size_t i = 0; i != leaves.size(); ++i)
        {
            geometry::Point3 normal = geometry::Point3::Zero();
            for(size_t j = 0; j != leaves.size(); ++j)
            {
                if(basis_funcs.coeff(i, j) > 0)
                normal += basis_funcs.coeff(i, j) * weighted_normal[leaves[j]->id];
            }
            leaves[i]->normal = normal;
            if(normal.norm() > max_length)
            max_length = normal.norm();
        }
        for(size_t i = 0; i != leaves.size(); ++i)
        {
            leaves[i]->normal /= max_length;
        }
#if 1
        /* debug*/
        geometry::PointCloud ocpcd = *(octree.GetPointCloud());
        ocpcd.WriteToPLY("octree_pcd.ply");
#endif
        // compute the divergence of normal
        
        // construct the L matrix, which is to compute the laplacian of X
        
        // solve poisson equation

    }
}
}