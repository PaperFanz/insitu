#include "view.hpp"

namespace insitu_common {

View::View(std::string _name, std::string _img_src)
{
    name = _name;
    img_src =  _img_src;
    img_type = OTHER;
    filters = std::vector<Filter>();
}

View::~View()
{
    filters.erase(filters.begin(), filters.begin() + filters.size());
}

const std::string View::getName()
{
    return name;
}

}