#ifndef insitu_plugins_Crosshair_HPP
#define insitu_plugins_Crosshair_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>

namespace insitu_plugins
{
class Crosshair : public insitu::Filter
{
public:
    Crosshair(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void onInit(void);

    void onDelete(void);

    int color = 0;

};    // end class Crosshair

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Crosshair_HPP
