#include "ColorMapping.h"
namespace dragon
{
namespace tool
{
    //blue-> white-> red (0 0 1-> 1 1 1-> 1 0 0), 255 * 2/
    void ColorRemapping(const std::vector<double> &values, geometry::Point3List &mapped_color)
    {
        mapped_color.clear();
        double min_value = std::numeric_limits<double>::max();
        double max_value = std::numeric_limits<double>::lowest();
        for(size_t i = 0; i != values.size(); ++i)
        {
            if(values[i] < min_value) min_value = values[i];
            if(values[i] > max_value) max_value = values[i];
        }

        std::vector<int> histogram(COLOR_RANGE, 0);
        std::vector<int> color_id;
        for(size_t i = 0; i != values.size(); ++i)
        {
            size_t index = (values[i] - min_value) / (max_value - min_value) * (COLOR_RANGE - 1);
            color_id.push_back(index);
            histogram[index] ++;
        }
        std::vector<int> map(COLOR_RANGE, 0);
        size_t current_n = 0;
        geometry::Point3List color_table;
        for(size_t i = 0; i != COLOR_RANGE; ++i)
        {
            if(i < COLOR_RANGE / 2)
            {
                color_table.push_back( geometry::Point3((i*2 + 0.0) / COLOR_RANGE, (i * 2 +0.0)/ COLOR_RANGE, 1));
                            // std::cout<<i<<" "<<color_table.back().transpose()<<std::endl;
            }
            else
            {
                color_table.push_back( geometry::Point3(1, 2.0 * (COLOR_RANGE - i ) / COLOR_RANGE, 2.0 * (COLOR_RANGE - i) / COLOR_RANGE));
            }

        }
        for(size_t i = 0; i != COLOR_RANGE; ++i)
        {
            //std::cout<<histogram[i]<<" "<<values.size()<<std::endl;
            current_n += histogram[i];
            map[i] = (COLOR_RANGE - 1) * (current_n + 0.0) / values.size();
        }
        for(size_t i = 0; i != values.size(); ++i)
        {
            mapped_color.push_back(color_table[color_id[i]]);
        }
    }
}
}