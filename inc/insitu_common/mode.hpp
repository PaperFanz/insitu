/*
    mode.hpp - definition of the mode class
*/

#ifndef mode_h

#include <map>
#include <string>

#include "view.hpp"

namespace insitu_common {

class Mode
{
private:
    std::string name;
    std::map<std::string, View> views;
    
public:
    Mode(std::string mode_name);
    ~Mode();
};

}

#endif
