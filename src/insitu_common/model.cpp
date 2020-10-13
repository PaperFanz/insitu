#include "model.hpp"

namespace insitu_common {

Model::Model(std::string _filename)
{
    filename = _filename;
    modes = std::map<std::string, insitu_common::Mode>();
}

Model::~Model()
{
}

bool Model::addMode(insitu_common::Mode newmode)
{
    auto pos = modes.find(newmode.getName());
    if (pos == modes.end()) {
        modes.insert({newmode.getName(), newmode});
        return true;
    } else {
        // mode of same name exists
        return false;
    }
}

bool Model::rmMode(std::string modename)
{
    auto pos = modes.find(modename);
    if (pos == modes.end()) {
        // no such mode
        return false;
    } else {
        modes.erase(pos);
        return true;
    }
}

}