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
}
}