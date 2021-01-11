#include "filter_factory.hpp"

namespace insitu {

FilterFactory::FilterFactory(const std::string& pkg)
{
    nLoader = new nodelet::Loader(boost::bind(&FilterFactory::create_instance, this, _1));
    pLoader = new pluginlib::ClassLoader<insitu::Filter>(pkg, "insitu::Filter");
}

FilterFactory::~FilterFactory(void)
{
    if (nLoader) delete nLoader;
    if (pLoader) delete pLoader;
}

std::vector<std::string>
FilterFactory::getFilterList(void)
{
    return pLoader->getDeclaredClasses();
}

boost::shared_ptr<insitu::Filter>
FilterFactory::loadFilter(const std::string& filter, const std::string& name)
{
    nodelet::M_string rmap_;
    nodelet::V_string argv_;

    instance_.reset();

    /* this will call create_instance and set instance_ */
    bool loaded = nLoader->load(name, filter, rmap_, argv_);

    if (loaded) {
        // TODO track
    } else {
        // TODO error
        throw std::runtime_error("Failed to load Filter: " + filter);
    }

    // pass ownership off to the caller
    boost::shared_ptr<insitu::Filter> instance = instance_;
    instance_.reset();
    return instance;
}

boost::shared_ptr<nodelet::Nodelet>
FilterFactory::create_instance(const std::string& lookup_name)
{
    instance_ = pLoader->createInstance(lookup_name);
    return instance_;
}

bool
FilterFactory::unloadFilter(const std::string& name)
{
    return nLoader->unload(name);
}

std::string
FilterFactory::getClassDescription(const std::string& name)
{
    return pLoader->getClassDescription(name);
}

std::string
FilterFactory::getClassPackage(const std::string& name)
{
    return pLoader->getClassPackage(name);
}

std::string
FilterFactory::getName(const std::string& name)
{
    return pLoader->getName(name);
}

}
