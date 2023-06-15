#ifndef DRAGON_KDTREE_H
#define DRAGON_KDTREE_H
#include "Geometry/BasicGeometry.h"
#include "nanoflann.hpp"
#include <memory>
//use nanoflann library
namespace dragon
{
namespace geometry
{
    //kdtree index
    class SearchParameter
    {
        public:
        //kdtree parameter
        //is sorted
        SearchParameter(int _checks = 256, 
            float _eps = 1e-8, bool _sorted = true)
        {
            checks = _checks;
            eps = _eps;
            sorted = _sorted;
        }
        bool sorted = true;
        //eps
        float eps = 1e-8;
        //recursively check
        int checks = 32;
        

    };

    template <int _D>
    struct NanoPointList
    {
        
        geometry::PointList<_D> point_list;

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return point_list.size(); }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        //  "if/else's" are actually solved at compile time.
        inline geometry::scalar kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            return point_list[idx][dim];
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    };
// nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, one_piece::geometry::NanoPointList<3>, float>, 
//     one_piece::geometry::NanoPointList<3>, 3, long unsigned int>::KDTreeSingleIndexAdaptor()
    template<int T = 3>
    class KDTree
    {
        public:
        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<geometry::scalar, NanoPointList<T> > ,
            NanoPointList<T>,
            T
            > NanoKDTree;
        typedef std::shared_ptr<NanoKDTree> NanoKDTreePtr; 
        KDTree(int _max_leaf = 10):max_leaf(_max_leaf)
        {
            //kdtree = NanoKDTree();
        }
        void BuildTree(const geometry::PointXList &points)
        {
            nano_point_list.point_list.clear();
            for(size_t i = 0; i != points.size(); ++i)
            nano_point_list.point_list.push_back(points[i]);
            kdtree_ptr = NanoKDTreePtr( new NanoKDTree(T /*dim*/, nano_point_list, 
                nanoflann::KDTreeSingleIndexAdaptorParams(max_leaf /* max leaf */) ));  
            kdtree_ptr->buildIndex();
        }
        void BuildTree(const geometry::PointList<T> &points)
        {
            nano_point_list.point_list = points;
            kdtree_ptr = NanoKDTreePtr( new NanoKDTree(T /*dim*/, nano_point_list, 
                nanoflann::KDTreeSingleIndexAdaptorParams(max_leaf /* max leaf */) ));
            kdtree_ptr->buildIndex();
        }
        void RadiusSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, double radius, size_t max_result = 100, 
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            RadiusSearch(point, _indices, dists, radius, max_result, sp);
            indices.resize(_indices.size());
            for(size_t i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void RadiusSearch(const geometry::VectorX &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, double radius, size_t max_result = 100, 
            const SearchParameter &sp = SearchParameter())
        {
            if(point.rows() != T)
            {
                std::cout<<RED<<"[ERROR]::[RadiusSearch]::Wrong dimension!"<<RESET<<std::endl;
                return;
            }
            geometry::Vector<T> _point = point;
            RadiusSearch(_point, indices, dists, radius, max_result, sp);
        }
        void RadiusSearch(const geometry::Vector<T> &point, std::vector<int> &indices, 
            std::vector<float > &dists, double radius, size_t max_result = 100, 
            const SearchParameter sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            RadiusSearch(point, _indices, dists, radius, max_result, sp);
            indices.resize(_indices.size());
            for(size_t i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void RadiusSearch(const geometry::Vector<T> &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, double radius, size_t max_result = 100, 
            const SearchParameter sp = SearchParameter())
        {
            std::vector<std::pair<size_t, geometry::scalar> >   ret_matches;
            //std::cout<<"max result: "<<max_result<<std::endl;
            size_t search_num = kdtree_ptr->radiusSearch(point.data(), radius, ret_matches, max_result * 2.5,
                nanoflann::SearchParams(sp.checks, sp.eps, sp.sorted));
            if(search_num > max_result)
            search_num = max_result;
            indices.resize(search_num);
            dists.resize(search_num);

            for (size_t i = 0; i < search_num; ++i)
            {   
                indices[i] = static_cast<float>(ret_matches[i].first);
                dists[i] = ret_matches[i].second;
            }
        }
        void KnnSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            KnnSearch(point, _indices, dists, k, sp);
            indices.resize(_indices.size());
            for(size_t i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void KnnSearch(const geometry::VectorX &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            if(point.rows() != T)
            {
                std::cout<<RED<<"[ERROR]::[KnnSearch]::Wrong dimension!"<<RESET<<std::endl;
                return;
            }
            geometry::Vector<T> _point = point;
            KnnSearch(_point, indices, dists, k, sp);

        }
        void KnnSearch(const geometry::Vector<T> &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            KnnSearch(point, _indices, dists, k, sp);
            indices.resize(_indices.size());
            for(size_t i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void KnnSearch(const geometry::Vector<T> &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            indices.resize(k);
            geometry::ScalarList out_dist_sqr(k);

            const size_t search_num = kdtree_ptr->knnSearch(point.data(), k, &indices[0], &out_dist_sqr[0]);
            
            // In case of less points in the tree than requested: if that happen, we just use searched result.
            // ret_index.resize(num_results);
            // out_dist_sqr.resize(num_results);
            indices.resize(search_num);
            dists.resize(search_num);

            for (size_t i = 0; i < search_num; ++i)
            {
                dists[i] = out_dist_sqr[i];
            }           
        }

        void KnnRadiusSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, float radius,
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            KnnRadiusSearch(point, _indices, dists, k, radius, sp);
            indices.resize(_indices.size());
            for(size_t i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void KnnRadiusSearch(const geometry::VectorX &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, int k, float radius,
            const SearchParameter &sp = SearchParameter())
        {
            if(point.rows() != T)
            {
                std::cout<<RED<<"[ERROR]::[KnnSearch]::Wrong dimension!"<<RESET<<std::endl;
                return;
            }
            geometry::Vector<T> _point = point;
            KnnRadiusSearch(_point, indices, dists, k, radius, sp);

        }
        void KnnRadiusSearch(const geometry::Vector<T> &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, float radius,
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            KnnRadiusSearch(point, _indices, dists, k, radius, sp);
            indices.resize(_indices.size());
            for(size_t i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void KnnRadiusSearch(const geometry::Vector<T> &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, int k, float radius,
            const SearchParameter &sp = SearchParameter())
        {
            indices.resize(k);
            geometry::ScalarList out_dist_sqr(k);

            const size_t search_num = kdtree_ptr->knnSearch(point.data(), k, &indices[0], &out_dist_sqr[0]);
            
            // In case of less points in the tree than requested: if that happen, we just use searched result.
            // ret_index.resize(num_results);
            // out_dist_sqr.resize(num_results);
            indices.resize(search_num);
            dists.resize(search_num);
            for (size_t i = 0; i < search_num; ++i)
            {
                dists[i] = out_dist_sqr[i];
            }      
            size_t in_radius = 0;
            for(; in_radius != search_num; ++in_radius)
            {
                // std::cout<<"radius: "<<dists[in_radius] <<std::endl;
                if(dists[in_radius] > radius) break;
            }
            indices.resize(in_radius);
            dists.resize(in_radius);
        }
        protected:
        NanoKDTreePtr kdtree_ptr;
        int max_leaf = 10;
        NanoPointList<T> nano_point_list;
    };
    // dim at runtime
    class KDTreeEigenMatrix
    {
        public:
        typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<geometry::scalar, 
            Eigen::Dynamic, Eigen::Dynamic>> NanoKDTreeEigenMatrix;

        typedef std::shared_ptr<NanoKDTreeEigenMatrix> NanoKDTreeEigenMatrixPtr; 
        KDTreeEigenMatrix(int _dim, int _max_leaf = 10):dim(_dim), max_leaf(_max_leaf)
        {
            //kdtree = NanoKDTreeEigenMatrix();
        }
        void BuildTree(const geometry::MatrixX &points)
        {
            kdtree_ptr = NanoKDTreeEigenMatrixPtr(new NanoKDTreeEigenMatrix (dim, std::cref(points), max_leaf));
            kdtree_ptr->index->buildIndex();
        }
        void KnnSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            if(point.rows() != dim)
            {
                // std::cout<<point.rows()<<" "<<dim<<std::endl;
                std::cout<<RED<<"[ERROR]::[KnnSearch]::Wrong dimension!"<<RESET<<std::endl;
                return;
            }
            std::vector<size_t> _indices;
            KnnSearch(point, _indices, dists, k, sp);
            indices.resize(_indices.size());
            for(size_t i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void KnnSearch(const geometry::VectorX &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            if(point.rows() != dim)
            {
                std::cout<<RED<<"[ERROR]::[KnnSearch]::Wrong dimension!"<<RESET<<std::endl;
                return;
            }
            nanoflann::KNNResultSet<geometry::scalar> result_set(k);
            indices.resize(k);
            dists.resize(k);
            geometry::ScalarList out_dist_sqr(k);
            result_set.init(&indices[0], &out_dist_sqr[0]);
            kdtree_ptr->index->findNeighbors(result_set, point.data(),
                                            nanoflann::SearchParams(sp.checks, sp.eps, sp.sorted));

            for (int i = 0; i < k; ++i)
            {
                dists[i] = out_dist_sqr[i];
            }           
        }

        void SetDim(const int _dim)
        {
            dim = _dim;
        }
        protected:
        NanoKDTreeEigenMatrixPtr kdtree_ptr;
        int dim;
        int max_leaf = 10;

    };
}
}
#endif

