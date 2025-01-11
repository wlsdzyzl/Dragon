#ifndef POINT_H
#define POINT_H
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include "IO/ConsoleColor.h"
#include <iostream>
#include <random>
#define DRAGON_EPS 1e-6
namespace dragon
{
namespace geometry
{
#ifdef USING_FLOAT64
    typedef double scalar;
#else
    typedef float scalar;
#endif
    typedef std::vector<scalar> ScalarList;
    typedef Eigen::Matrix<scalar, 2, 1> Point2;
    typedef Eigen::Matrix<scalar, 3, 1> Point3;
    typedef Eigen::Matrix<scalar, 2, 1> Vector2;
    typedef Eigen::Matrix<scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<scalar, 4, 1> Vector4;
    typedef Eigen::Matrix<scalar, 6, 1> Vector6;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, 1> VectorX;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, 1> PointX;
    typedef Eigen::Matrix<scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<scalar, 4, 4> Matrix4;
    typedef Eigen::Matrix<scalar, 6, 6> Matrix6;
    typedef Eigen::Matrix<scalar, 3, 6> Matrix3X6;
    
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    template <int T>
        using Vector = Eigen::Matrix<scalar, T, 1>;

    typedef Eigen::Matrix<int, 2, 1> Point2i;
    typedef Eigen::Matrix<int, 3, 1> Point3i;
    typedef Eigen::Matrix<unsigned int, 2, 1> Point2ui;
    typedef Eigen::Matrix<unsigned int, 3, 1> Point3ui;
    typedef std::vector<Point2i, Eigen::aligned_allocator<Point2i> > Point2iList;
    typedef std::vector<Point3i, Eigen::aligned_allocator<Point3i> > Point3iList;
    typedef std::vector<Point2ui, Eigen::aligned_allocator<Point2ui> > Point2uiList;
    typedef std::vector<Point3ui, Eigen::aligned_allocator<Point3ui> > Point3uiList;
    
    typedef std::vector<Point2, Eigen::aligned_allocator<Point2> > Point2List;
    typedef std::vector<Point3, Eigen::aligned_allocator<Point3> > Point3List; 
    typedef std::vector<PointX, Eigen::aligned_allocator<PointX> > PointXList; 
    typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3> > Vec3List;
    typedef std::vector<Vector4, Eigen::aligned_allocator<Vector4> > Vec4List;
    template <int T>
        using PointList = std::vector<Eigen::Matrix<scalar, T, 1>, 
            Eigen::aligned_allocator<Eigen::Matrix<scalar, T, 1>>>;

    typedef std::vector<Matrix4, Eigen::aligned_allocator<Matrix4> > Mat4List;
    typedef std::vector<MatrixX, Eigen::aligned_allocator<MatrixX> > MatXList;
    typedef Matrix4 TransformationMatrix;
    //lie group and lie algebra
    //SE3 and se3
    // typedef Vector6 Se3;
    // typedef Vector3 So3;
    // typedef Matrix4 SE3;
    // typedef Matrix3 SO3;
    // typedef Mat4List SE3List;
    // Vector6 SE3ToSe3(const Matrix4 &input);
    // Matrix4 Se3ToSE3(const Vector6 &input);
    Point3List TransformPoints(const Matrix4 &T, const Point3List &points);
    Point3 TransformPoint(const Matrix4 &T, const Point3 &point);
    Vec3List TransformNormals(const Matrix4 &T, const Vec3List &normals);
    Vector3 TransformNormal(const Matrix4 &T, const Vector3 &normal);

    TransformationMatrix RandomTransformation();
    double ComputeTriangleArea(const PointX &a, const PointX &b, const PointX &c);
    // double inline cos(const Vector3 &a, const Vector3 &b){return a.dot(b) / (a.norm() * b.norm()); }
    // double inline sin(const Vector3 &a, const Vector3 &b){double cos_= cos(a, b); return sqrt(1-cos_ * cos_) ; }
    // double inline tan(const Vector3 &a, const Vector3 &b){return sin(a, b) / cos(a, b);}
    // double inline cot(const Vector3 &a, const Vector3 &b){return 1 / tan(a, b);}
    double inline Cos(const VectorX &a, const VectorX &b)
    {
        double norm_prod = (a.norm() * b.norm());
        if(norm_prod < DRAGON_EPS)
            norm_prod = 2 * DRAGON_EPS;
        return a.dot(b) / norm_prod; 
    }
    double inline Sin(const VectorX &a, const VectorX &b){double cos_= Cos(a, b); return sqrt(1-cos_ * cos_) ; }
    double inline Tan(const VectorX &a, const VectorX &b)
    { 
        double cos_ = Cos(a, b); 
        if(std::fabs(cos_) < DRAGON_EPS) 
            cos_ = cos_ > 0 ? DRAGON_EPS * 2:-DRAGON_EPS * 2;
        return Sin(a, b) / cos_;}
    double inline Cot(const VectorX &a, const VectorX &b)
    { 
        double tan_ = Tan(a,b); 
        if(std::fabs(tan_)<DRAGON_EPS) 
            tan_ = tan_ > 0? DRAGON_EPS * 2: - DRAGON_EPS * 2;
        return 1 / tan_;
    }
    double inline ClampCot(double cot_value)
    {
        double bound = 19.1; // 3 degrees
        return (cot_value < -bound ? -bound : (cot_value > bound ? bound : cot_value));
    }
    double inline ClampCot(const VectorX &a, const VectorX &b){return ClampCot(Cot(a, b));}
    // compute the circum center of triangle
    std::tuple<Point3, double> CircumCenter(const Point3 &a, const Point3 &b, const Point3 &c);
    std::tuple<Point2, double, double> CircumCenter(const Point2 &a, const Point2 &b, const Point2 &c);
    int TriangleType(const PointX &a, const PointX &b, const PointX &c);
    double AngleOfVector(const VectorX &a, const VectorX &b);
    int AngleType(const VectorX &a, const VectorX &b);
    void AddToCoefficientTriplet(std::vector<Eigen::Triplet<scalar>> &coefficients,
        int start_row, int start_col, const MatrixX &JTJ);
    Matrix3 RotationMatrix(const Vector3 &axis, double angle);
    Matrix3 GetSkewSymmetricMatrix(const Vector3 &t);
    std::tuple<Point3, double ,double> FitPlane(const Point3List & _points);
    std::tuple<Vector2, double ,double> FitLine(const Point2List & _points);
    // computational geometry
    // from onepiece geometry2d
    struct LineSegment;
    struct Line;

    double Cross3(const Point2 &a, const Point2 &b, const Point2 &c);
    bool InSegBounding(const LineSegment &l, const Point2 &p);
    bool InSegBoundingX(const LineSegment &l, const Point2 &p);
    bool InSegBoundingY(const LineSegment &l, const Point2 &p);
    bool IsIntersecting(const LineSegment &l1, const LineSegment &l2);
    bool IsIntersecting(const Line & l1, const LineSegment &l2);
    Line LineFromSeg(const LineSegment &s);
    Point2 LineIntersect(const Line &a, const Line &b);
    Point2 SegIntersect(const LineSegment &s1, const LineSegment &s2);
    Point2 LineSegIntersect(const Line &a, const LineSegment & b);
    double Distance(const Point2 &a, const Point2 &b);
    double Distance(const Point3 &a, const Point3 &b);
    Point2 ProjectionPointToLine(const Line &a, const Point2 &p);
    Point2 ProjectionPointToLineSegment(const LineSegment &a, const Point2 &p);
    int CheckPointInConvexPoly(const geometry::Point2List &points, const Point2 &p);
    int CheckPointToLine(const Line &line, const Point2 &point);
    int CheckPointToLine(const Point2 &a, const Point2 &b, const Point2 &z );
    int CheckPointInTriangle(const Point2 & a, const Point2 &b, const Point2 & c, const Point2 &p);
    int CheckPointProjectionInTriangle(const Point3& query_point,
                        const Point3& triangle_vertex_0,
                        const Point3& triangle_vertex_1,
                        const Point3& triangle_vertex_2, Point3 &projected_p);
    int CheckPointProjectionOnLineSegment(const Point3 &query_point, const Point3 &start, const Point3 &end,
                        Point3 &projected_p);
    double ComputeAreaTriangle(Point2 a, Point2 b, Point2 c);
    double ComputeAreaConvexPoly(const Point2List &points);
    geometry::Matrix3 RotationMatrixBetweenVectors(const Vector3 &a, const Vector3 &b);
    Line VerticalBisector(const Point2 & a, const Point2 &b);
    std::vector<std::vector<size_t>> VoxelClustering(const Point3List &pcd, double grid_len);
    std::vector<std::vector<size_t>> RadiusClustering(const Point3List &pcd, const ScalarList & radius, double search_factor = 0.5);
    Point3i GetGridIndex(const geometry::Point3 &points, double grid_len);
    struct LineSegment
    {
        Point2 p0, p1;
        LineSegment(){}
        LineSegment(Point2 _p0, Point2 _p1) : p0(_p0), p1(_p1) {}
        double Length()
        {
            return Distance(p0, p1);
        }
    };

    struct Line
    {
        geometry::Vector2 n;
        double d;
        Line() = default;
        Line(double a, double b, double c)
        {
            n(0) = a;
            n(1) = b;
            d = c;
        }
        Line(const geometry::Vector2 &_n, double _d)
        {
            n = _n;
            d = _d;
        }
        Line(const geometry::Point3 &l)
        {
            n = l.head<2>();
            d = l(2);
        }
    };
    // for hash
    struct PairHasher
    {
        template<class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2>& p) const
        {
            // auto h1 = std::hash<T1>{}(p.first);
            // auto h2 = std::hash<T2>{}(p.second);
            // return h1 ^ h2;
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            return ( p.first * p1 ^ p.second * p2 );
        }
    };
    struct PixelGridHasher
    {
        std::size_t operator() (const Point2i& key) const
        {
            // auto h1 = std::hash<T1>{}(p.first);
            // auto h2 = std::hash<T2>{}(p.second);
            // return h1 ^ h2;
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            return ( key(0) * p1 ^ key(1) * p2);
        }
    };

    struct VoxelGridHasher
    {
            // Three large primes are used for spatial hashing.
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            static constexpr size_t p3 = 83492791;

            std::size_t operator()(const Point3i& key) const
            {
                return ( key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
            }
    };

    //linear solver
    geometry::VectorX SolveBySVD(const geometry::MatrixX &A, const geometry::VectorX &b);
    geometry::MatrixX SolveByLu(const geometry::MatrixX &A, const geometry::MatrixX &b);
    geometry::VectorX SolveByThomas(const geometry::MatrixX &A, const geometry::VectorX &b);
}
}
#endif