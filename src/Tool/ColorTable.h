#ifndef COLOR_TABLE_H
#define COLOR_TABLE_H
#include "Geometry/BasicGeometry.h"
namespace dragon
{
namespace tool
{
static geometry::Point3List COLOR_TABLE = {
            geometry::Point3(150, 150, 150) / 255.0,
            geometry::Point3(174, 199, 232) / 255.0,     // wall
            geometry::Point3(152, 223, 138) / 255.0,     // floor
            geometry::Point3(31, 119, 180) / 255.0,      // cabinet
            geometry::Point3(255, 187, 120) / 255.0,     // bed
            geometry::Point3(188, 189, 34) / 255.0,      // chair
            geometry::Point3(140, 86, 75) / 255.0,       // sofa
            geometry::Point3(255, 152, 150) / 255.0,     // table
            geometry::Point3(214, 39, 40) / 255.0,       // door
            geometry::Point3(197, 176, 213) / 255.0,     // window
            geometry::Point3(148, 103, 189) / 255.0,     // bookshelf
            geometry::Point3(196, 156, 148) / 255.0,     // picture
            geometry::Point3(23, 190, 207) / 255.0,      // counter
            geometry::Point3(178, 76, 76) / 255.0,
            geometry::Point3(247, 182, 210) / 255.0,     // desk
            geometry::Point3(66, 188, 102) / 255.0,
            geometry::Point3(219, 219, 141) / 255.0,     // curtain
            geometry::Point3(140, 57, 197) / 255.0,
            geometry::Point3(202, 185, 52) / 255.0,
            geometry::Point3(51, 176, 203) / 255.0,
            geometry::Point3(200, 54, 131) / 255.0,
            geometry::Point3(92, 193, 61) / 255.0,
            geometry::Point3(78, 71, 183) / 255.0,
            geometry::Point3(172, 114, 82) / 255.0,
            geometry::Point3(255, 127, 14) / 255.0,      // refrigerator
            geometry::Point3(91, 163, 138) / 255.0,
            geometry::Point3(153, 98, 156) / 255.0,
            geometry::Point3(140, 153, 101) / 255.0,
            geometry::Point3(158, 218, 229) / 255.0,     // shower curtain
            geometry::Point3(100, 125, 154) / 255.0,
            geometry::Point3(178, 127, 135) / 255.0,
            geometry::Point3(120, 185, 128) / 255.0,
            geometry::Point3(146, 111, 194) / 255.0,
            geometry::Point3(44, 160, 44) / 255.0,       // toilet
            geometry::Point3(112, 128, 144) / 255.0,     // sink
            geometry::Point3(96, 207, 209) / 255.0,
            geometry::Point3(227, 119, 194) / 255.0,     // bathtub
            geometry::Point3(213, 92, 176) / 255.0,
            geometry::Point3(94, 106, 211) / 255.0,
            geometry::Point3(82, 84, 163) / 255.0,       // otherfurn
            geometry::Point3(100, 85, 144)/ 255.0 };  
}}
#endif