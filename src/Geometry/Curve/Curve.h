#ifndef DRAGON_CURVE_H
#define DRAGON_CURVE_H
#include "Geometry/BasicGeometry.h"
#include "Tool/CppExtension.h"
#include "Geometry/Parameterization.h"
namespace dragon
{
namespace geometry
{
namespace curve
{  
    template<int T>
        geometry::Vector<T> deCasteljau_recur(const geometry::PointList<T> & points, geometry::scalar t)
    {
        if(points.size() == 1)
            return points[0];
        geometry::PointList<T> new_points;
        for(size_t i = 0; i < points.size() - 1; ++i)
            new_points.push_back((1 - t) * points[i] + t * points[i+1]);
        return deCasteljau_recur(new_points, t);
    }
    template<int T>
        geometry::PointX deCasteljau(const geometry::PointList<T> & points, geometry::scalar t)
    {
        if(t > 1.0f || t < 0)
        {
            std::cout<<YELLOW<<"[WARNING]::[deCasteljau]::t should be in range of [0, 1]."<<RESET<<std::endl;
            return points[0];
        }
        else
        return deCasteljau_recur<T>(points, t);
    }
    template<int T>
        geometry::PointList<T> Bezier(const geometry::PointList<T> &control_points, int n = 1000)
    {
        geometry::ScalarList t = tool::LinSpace(0.0, 1.0, n);
        geometry::PointList<T> res;
        for(size_t i = 0; i != t.size(); ++i)
        res.push_back(deCasteljau(control_points, t[i]));
        return res;
    }
    template<int T>
        geometry::PointList<T> BezierInterpolation(const geometry::PointList<T> &inter_points, int n = 1000)
    {
        std::vector<geometry::PointList<T>> bezier_control_point_list(inter_points.size() - 1);
        geometry::ScalarList global_t = tool::LinSpace(0.0, 1.0, n);
        geometry::ScalarList t_signal = parameterization::Chordal<T>(inter_points);
        // generate bezier_control_points
        geometry::PointList<T> res;
        if(inter_points.size() < 3) 
        {
            std::cout<<YELLOW<<"[WARNING]::[BezierInterpolation]::points size should be larger than 2."<<RESET<<std::endl;
            return inter_points;
        }
        for(size_t i = 0; i != inter_points.size() - 1; ++i)
        bezier_control_point_list[i].push_back(inter_points[i]);

        for(size_t i = 0; i != inter_points.size() - 2; ++i)
        {
            geometry::Vector<T> dist = (inter_points[i+2] - inter_points[i])/6;
            bezier_control_point_list[i].push_back(inter_points[i+1] - dist);
            bezier_control_point_list[i+1].push_back(inter_points[i+1] + dist);
        }
        for(size_t i = 1; i != inter_points.size(); ++i)
        bezier_control_point_list[i - 1].push_back(inter_points[i]);
        //based on t_signal to determine the group
        size_t current_t_signal = 0;
        for(size_t i = 0; i != global_t.size(); ++i)
        {
            if(global_t[i] > t_signal[current_t_signal + 1])
                current_t_signal += 1;
            geometry::scalar local_t = (global_t[i] - t_signal[current_t_signal])/ ( t_signal[current_t_signal + 1] - t_signal[current_t_signal]);
            if(local_t > 1.0) local_t = 1.0;
            if(local_t < 0.0) local_t = 0.0;
            res.push_back(deCasteljau(bezier_control_point_list[current_t_signal], local_t));
        }
        return res;
    }
    geometry::MatrixX CubicSpline(const geometry::ScalarList &x, const geometry::ScalarList &y, int boundary_type = 0)
    {
        geometry::ScalarList h_vec; 
        for(size_t i = 0; i != x.size() - 1; ++i)
        {
            // std::cout<<x[i]<<" "<<y[i]<<std::endl;
            h_vec.push_back((x[i+1] - x[i]));
        }
        // std::cout<<x.back()<<" "<<y.back()<<std::endl;
        assert(x.size() == y.size());
        geometry::MatrixX A(x.size(), x.size());
        A.setZero();
        if(boundary_type == 0)
        {
            for(size_t i = 0; i != x.size(); ++i)
            {
                if(i == x.size() - 1 || i == 0) A(i, i) = 1; 
                else
                {
                    A(i, i-1) = h_vec[i - 1];
                    A(i, i+1) = h_vec[i];
                    A(i, i) = 2 * (h_vec[i] + h_vec[i-1]);
                } 
            }

        }
        else
        {
            for(size_t i = 0; i != x.size(); ++i)
            {
                if(i == 0)
                {
                    A(i, i) = 2 * h_vec[i];
                    A(i, i+1) = h_vec[i];
                }
                else if(i == x.size() - 1)
                {
                    A(i, i) = 2 * h_vec[i - 1];
                    A(i, i-1) = h_vec[i-1];
                }
                else
                {
                    A(i, i-1) = h_vec[i - 1];
                    A(i, i+1) = h_vec[i];
                    A(i, i) = 2 * (h_vec[i] + h_vec[i-1]);
                } 
            }
        }
        geometry::VectorX f(x.size());
        f.setZero();
        double second_tmp = 0.0;
        for(size_t i = 1; i < x.size() - 1; ++i)
        {
            if(i == 1) second_tmp = (y[1] - y[0]) / h_vec[0];
            double first_tmp = (y[i+1] - y[i])/h_vec[i];
            f(i) = 6 * (first_tmp - second_tmp);
            second_tmp = first_tmp;
        }
        // std::cout<<"A: "<<A<<std::endl;
        // std::cout<<"f: "<<f<<std::endl;
        // solve equation
        geometry::VectorX x_;
        x_ = geometry::SolveByThomas(A, f);
        // std::cout<<A * x_ - f<<std::endl;
        // compute a, b, c, d
        geometry::MatrixX res(4, h_vec.size());
        for(size_t i = 0; i != h_vec.size(); ++i)
        {
            res(0, i) = y[i];
            res(1, i) = (y[i+1] - y[i]) / h_vec[i] - x_(i)*h_vec[i] / 2 - (x_(i+1) - x_(i)) *h_vec[i]/ 6;
            res(2, i) = x_(i) / 2;
            res(3, i) = (x_(i+1) - x_(i)) / (6 * h_vec[i]);
        }
        // std::cout<<res<<std::endl;
        return res;
    }    
    template<int T>
        geometry::PointList<T> CubicSpline(const geometry::PointList<T> &inter_points, int n = 1000, int boundary_type = 0)
    {
        geometry::ScalarList t = parameterization::Chordal<T>(inter_points);
        std::vector<geometry::ScalarList> mul_p(T);
        for(int i = 0; i != T; ++i)
        {
            for(size_t j = 0; j != inter_points.size(); ++j)
            mul_p[i].push_back(inter_points[j](i));
        }
        geometry::MatXList coefficients;
        geometry::ScalarList global_t = tool::LinSpace(0.0, 1.0, n);
        for(int i = 0; i != T; ++i)
        {
            coefficients.push_back(CubicSpline(t, mul_p[i], boundary_type));
        }
        // use coefficients to compute multiple dimension value
        // compact all dimension values into one vector and get result 
        size_t current_group = 0;
        geometry::PointList<T> res;
        for(size_t i = 0; i != global_t.size(); ++i)
        {
            if(global_t[i] > t[current_group + 1])
                current_group += 1;
            double local_t = global_t[i] - t[current_group];
            // if(local_t > 1.0) local_t = 1.0;
            // if(local_t < 0.0) local_t = 0.0;
            geometry::Vector<T> p;
            geometry::Vector4 t_help(1, local_t, pow(local_t, 2), pow(local_t, 3));
            for(int j = 0; j != T; ++j)
            {
                p[j] = t_help.transpose() * coefficients[j].block<4, 1>(0, current_group) ; 
            }
            res.push_back(p);
        }        
        return res;
    }
        // through subdivision to get uniform cubic b spline 
    template<int T>
        geometry::PointList<T> BSpline(const geometry::PointList<T> &control_points, int n = 1000)
    {

    }
    template<int T>
        geometry::PointList<T> FourPointsInterpolation(const geometry::PointList<T> &inter_points, int n = 1000)
    {

    }
}
}
}

#endif