#ifndef DRAGON_COLOR_MAPPING_H
#define DRAGON_COLOR_MAPPING_H
#include "Geometry/BasicGeometry.h"
#define COLOR_RANGE 512
namespace dragon
{
namespace tool
{
    // blue-> white-> red (0 0 1-> 1 1 1-> 1 0 0), 255 * 2/
    // use histogram equalization
    void ColorRemapping(const geometry::ScalarList &values, geometry::Point3List &mapped_color);
}
}
#endif