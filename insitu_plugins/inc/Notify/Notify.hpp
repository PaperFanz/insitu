#ifndef insitu_plugins_Notify_HPP
#define insitu_plugins_Notify_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>

namespace insitu_plugins
{
class Notify : public insitu::Filter
{
public:
    Notify(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void filterInit(void);

    void onDelete(void);

};    // end class Notify

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Notify_HPP
