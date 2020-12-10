#ifndef DRAGON_PARAMETERIZATION_H
#define DRAGON_PARAMETERIZATION_H
#include "BasicGeometry.h"
namespace dragon
{
namespace geometry
{
namespace parameterization
{
    //1d parameterization
    template<int T>
        std::vector<double> Uniform(const geometry::PointList<T> &points)
    {
        double step = 1.0 / (points.size()-1);
        std::vector<double> results;
        for(size_t i = 0; i != points.size(); ++i)
        {
            results.push_back(i * step);
        }
        return results;
    }
    template<int T>
        std::vector<double> Chordal(const geometry::PointList<T> &points)
    {
        std::vector<double> dists;
        std::vector<double> results;
        double sum_dists = 0;
        for(size_t id = 1; id < points.size(); ++id)
        {
            dists.push_back( (points[id] - points[id - 1]).norm());
            sum_dists += dists.back();
        }
        results.push_back(0);
        for(auto d: dists)
            results.push_back(results.back() + d / sum_dists);
        return results;
    }
    template<int T>
        std::vector<double> Centripetal(const geometry::PointList<T> &points)
    {
        std::vector<double> dists;
        std::vector<double> results;
        double sum_dists = 0;
        for(size_t id = 1; id < points.size(); ++id)
        {
            dists.push_back( sqrt((points[id] - points[id - 1]).norm()));
            sum_dists += dists.back();
        }
        results.push_back(0);
        for(auto d: dists)
            results.push_back(results.back() + d / sum_dists);
        return results;
    }
    template<int T>
        std::vector<double> Foley(const geometry::PointList<T> &points)
    {
        std::vector<double> dists;
        std::vector<double> angles;
        std::vector<double> foley_dists;
        std::vector<double> results;
        double sum_dists = 0;
        double half_pi = M_PI/ 2.0;
        // get angles and dists
        for(size_t id = 1; id < points.size()-1; ++id)
        {
            auto v1 = points[id - 1] - points[id];
            auto v2 = points[id + 1] - points[id];
            double angle = AngleOfVector(v1, v2);
            angles.push_back( (angle > half_pi) ?(M_PI - angle): half_pi);
            if (id == 1)
                dists.push_back(v1.norm());
            dists.push_back(v2.norm());
        }
        foley_dists.push_back(dists[0] *( 1 +  1.5 * angles[0] * dists[1] /(dists[0] + dists[1]) ));
        for(size_t i = 1; i != points.size() - 2; ++i)
        {
            double d = dists[i] *( 1 +  1.5 * angles[i-1] * dists[i - 1] /( dists[i - 1] + dists[i]) + 
                1.5 * angles[i] * dists[i+1] /(dists[i] + dists[i + 1]));
            foley_dists.push_back(d);
            sum_dists += d;
        }
        size_t id = points.size() - 2;
        foley_dists.push_back(dists[id] *( 1 +  1.5 * angles[id - 1] * dists[id - 1] /( dists[id - 1] + dists[id]) ));
        sum_dists += foley_dists[0];
        sum_dists += foley_dists.back();
        results.push_back(0);
        for(auto d: foley_dists)
            results.push_back(results.back() + d / sum_dists);
        return results;           
    }
}
}
}
#endif