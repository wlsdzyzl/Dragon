#include "BasicGeometry.h"
#include <algorithm>
namespace dragon
{
namespace geometry
{
    class BoundingBox
    {
        public:
        geometry::Point3 Center()
        {
            return geometry::Point3(x_max + x_min, y_max + y_min, z_max + z_min) / 2;
        }
        double Radius()
        {
            geometry::Point3 center = Center();
            geometry::Point3 max_p(x_max, y_max, z_max);
            return (max_p - center).norm();
        }
        void AddPoint(const geometry::Point3 &p)
        {
            if(p(0) > x_max)
            x_max = p(0);
            if(p(0) < x_min)
            x_min = p(0);
            if(p(1) > y_max)
            y_max = p(1);
            if(p(1) < y_min)
            y_min = p(1);
            if(p(2) > z_max)
            z_max = p(2);
            if(p(2) < z_min)
            z_min = p(2);
        }
        double x_max = std::numeric_limits<double>::min();
        double x_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::min();
        double y_min = std::numeric_limits<double>::max();
        double z_max = std::numeric_limits<double>::min();
        double z_min = std::numeric_limits<double>::max();
    };
}
}