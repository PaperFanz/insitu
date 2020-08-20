#include "../include/insitu_common/model.hpp"

namespace insitu_common {

Model::Model(std::string _filename)
{
    filename = _filename;
    modes = std::map<std::string, insitu_common::Mode>();
}

Model::~Model()
{
}

void Model::addMode(insitu_common::Mode newmode)
{
    
}

}