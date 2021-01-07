#include "filter_factory.hpp"

namespace insitu {

FilterFactory::FilterFactory(std::string pkg)
{
    filterLoader = new pluginlib::ClassLoader<insitu::Filter>(pkg, "insitu::Filter");
}

std::vector<std::string> FilterFactory::getFilterList(void)
{
    return filterLoader->getDeclaredClasses();
}

boost::shared_ptr<insitu::Filter> FilterFactory::getInstance(std:: string filter)
{
    auto instance = filterLoader->createInstance(filter);

    nodelet::M_string rmap;
    nodelet::V_string argv;

    return instance;
}

}
