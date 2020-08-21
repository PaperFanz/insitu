/*
    view.hpp - definition of the view class
*/

#ifndef view_h
#define view_h

#include <string>
#include <vector>
#include <sensor_msgs/Image.h>

#include "filter.hpp"

namespace insitu_common {

typedef enum img_src_t {
    FILE,
    TOPIC,
    OTHER
} img_src_t;

class View
{
private:
    std::string name;
    std::string img_src;
    img_src_t img_type;
    std::vector<Filter> filters;
    
public:
    View(std::string _name, std::string _img_src);

    ~View();

    const std::string getName();
};

}

#endif
