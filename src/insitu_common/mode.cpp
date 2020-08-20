#include "../include/insitu_common/mode.hpp"

namespace insitu_common {

Mode::Mode(std::string mode_name)
{
    name = mode_name;
    views = std::map<std::string, View>(); 
}

Mode::~Mode()
{
}

}