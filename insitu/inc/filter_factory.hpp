#ifndef insitu_FILTER_FACTORY_HPP
#define insitu_FILTER_FACTORY_HPP

// plugin includes
#include <pluginlib/class_loader.h>
#include <insitu/filter.hpp>

namespace insitu {

class FilterFactory
{

private:
    // plugin loader
    pluginlib::ClassLoader<insitu::Filter> * filterLoader;

public:
    FilterFactory(std::string pkg);

    std::vector<std::string> getFilterList(void);

private:
    boost::shared_ptr<insitu::Filter> getInstance(std::string filter);

};

} // namespace insitu

#endif // insitu_FILTER_FACTORY_HPP
