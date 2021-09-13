#ifndef insitu_plugins_Transparent_HPP
#define insitu_plugins_Transparent_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>

namespace insitu_plugins
{
class Transparent : public insitu::Filter
{
public:
    Transparent(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void filterInit(void);

    void onDelete(void);

};    // end class Transparent

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Transparent_HPP
