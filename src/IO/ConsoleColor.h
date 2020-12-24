#ifndef CONSOLE_COLOR_H
#define CONSOLE_COLOR_H
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m" /* Yellow */
#define BLUE    "\033[34m" /* Blue */
#define PURPLE  "\033[35m" /* Purple */
#define D_GREEN "\033[36m" /* Dark Green */
#include <vector>
namespace dragon
{
namespace io
{
    static const std::vector<std::vector<int>> color_table = 
    {
        {0x1f, 0x77, 0xb4},
        {0xff, 0x7f, 0x0e},
        {0x2c, 0xa0, 0x2c},
        {0xd6, 0x27, 0x28},
        {0x94, 0x67, 0xbd},
        {0x8c, 0x56, 0x4b},
        {0xe3, 0x77, 0xc2},
        {0x7f, 0x7f, 0x7f},
        {0xbc, 0xbd, 0x22},
        {0x17, 0xbe, 0xcf}
    };
}
}
#endif