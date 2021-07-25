#ifndef insitu_FILTER_FACTORY_HPP
#define insitu_FILTER_FACTORY_HPP

// plugin includes
#include <nodelet/loader.h>
#include <pluginlib/class_loader.h>

// insitu includes
#include <insitu/filter.hpp>

namespace insitu
{
class FilterFactory
{
private:
    /* nodelet loader does the initialization */
    nodelet::Loader* nLoader;

    /* pluginlib loader is used to get a list of loadable insitu filters
       and do the actual loading */
    pluginlib::ClassLoader<insitu::Filter>* pLoader;

    /* variable for create_instance to store into */
    boost::shared_ptr<insitu::Filter> instance_;

    /* custom instance loader function for nodelet loader */
    boost::shared_ptr<nodelet::Nodelet>
    create_instance(const std::string& lookup_name);

public:
    FilterFactory();

    ~FilterFactory(void);

    std::vector<std::string> getFilterList(void);

    boost::shared_ptr<insitu::Filter> loadFilter(const std::string& filter,
                                                 const std::string& name);

    bool unloadFilter(const std::string& name);

    std::string getClassDescription(const std::string& name);

    std::string getClassPackage(const std::string& name);

    std::string getName(const std::string& name);
};

}    // namespace insitu

#endif    // insitu_FILTER_FACTORY_HPP
