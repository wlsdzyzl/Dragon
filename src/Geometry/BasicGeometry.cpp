#include "BasicGeometry.h"
namespace dragon
{
namespace geometry
{
    Matrix4 Se3ToSE3(const Vector6 &input)
    {
        auto tmp = Sophus::SE3Group<scalar>::exp(input);
        return tmp.matrix();
    }
    Vector6 SE3ToSe3(const Matrix4 &input)
    {
        Sophus::SE3Group<scalar>  tmp(input);
        return tmp.log(); 
    }
    Point3 TransformPoint(const Matrix4 &T, const Point3 &point)
    {
            Vector4 new_point =
                T *Vector4(point(0), point(1), point(2), 1.0);
            return new_point.head<3>() / new_point(3);        
    }
    void TransformPoints(const Matrix4 &T, Point3List &points)
    {
        for (auto& point : points) 
        {
            Vector4 new_point =
                T *Vector4(point(0), point(1), point(2), 1.0);
            point = new_point.head<3>() / new_point(3);
        }
    }
    void TransformNormals(const Matrix4 &T, Point3List &normals)
    {
        for (auto& normal : normals) 
        {
            Vector4 new_normal =
                T *Vector4(normal(0), normal(1), normal(2), 0.0);
            normal = new_normal.head<3>();
        }
    }  
    double ComputeTriangleArea(const Point3 &a, const Point3 &b, const Point3 &c)
    {
        auto edge1 = b-a;
        auto edge2 = c-a;

        // std::cout<<"area: "<<area<<" "<< <<std::endl;

        return 0.5 * edge1.norm() * edge2.norm() * Sin(edge1, edge2);
    }
    Point3 CircumCenter(const Point3 &a, const Point3 &b, const Point3 &c)
    {
        Point3 ac = c - a ;
        Point3 ab = b - a ;
        Point3 abXac = ab.cross( ac ) ;

        // this is the vector from a TO the circumsphere center
        Point3 toCircumsphereCenter = (abXac.cross( ab )*ac.squaredNorm() + ac.cross( abXac )*ab.squaredNorm()) / (2.f*abXac.squaredNorm()) ;
        // double circumsphereRadius = toCircumsphereCenter.norm() ;
        // The 3 space coords of the circumsphere center then:
        return a  +  toCircumsphereCenter ;
    }
    int TriangleType(const Point3 &a, const Point3 &b, const Point3 &c)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ba = -ab;
        Vector3 bc = c - b;
        Vector3 ca = -ac;
        Vector3 cb = -bc;
        double a1 = ac.dot(ab);
        double a2 = ba.dot(bc);
        double a3 = ca.dot(cb);
        // right angle
        if(std::fabs(a1) <EPS || std::fabs(a2) < EPS || std::fabs(a3) < EPS)
        return 1;
        // obtuse angle
        if(a1 < 0 || a2 < 0 || a3 < 0)
        return 2;
        // acute angle
        return 0;
    }
    int AngleType(const VectorX &a, const VectorX &b)
    {
        double dot = a.dot(b);
        if(std::fabs(dot) < EPS)
        return 1;
        if(std::fabs(dot) < 0)
        return 2;
        return 0;
    }  
    double AngleOfVector(const VectorX &a, const VectorX &b)
    {
        return std::acos(a.dot(b) / (a.norm() * b.norm()));
    }
    void AddToCoefficientTriplet(std::vector<Eigen::Triplet<scalar>> &coefficients,
        int start_row, int start_col, const MatrixX &JTJ)
    {
        int rows = JTJ.rows();
        int cols = JTJ.cols();
        for(int i = 0; i != rows; ++i)
        {
            for(int j = 0; j != cols; ++j)
            {
                if(JTJ(i, j) != 0)
                    coefficients.push_back(Eigen::Triplet<scalar>(i + start_row, j + start_col, JTJ(i,j)));
            }
        }
    }
    Matrix3 RotationMatrix(const Vector3 &_axis, double angle)
    {
        //Rodrigues
        // Matrix3 rotation_matrix;
        Vector3 axis = _axis.normalized();
        // Matrix3 aat =  axis * axis.transpose();
        // Matrix3 A = GetSkewSymmetricMatrix(axis);
        // rotation_matrix = aat + (Matrix3::Identity()-aat )*cos(angle) + sin(angle) * A;
        return Eigen::AngleAxis<geometry::scalar>(angle, axis).toRotationMatrix();
    }
    Matrix3 GetSkewSymmetricMatrix(const Vector3 &t)
    {
        Matrix3 t_hat;
        t_hat << 0, -t(2), t(1),
                t(2), 0, -t(0),
                -t(1), t(0), 0;
        return t_hat;        
    }
}
}