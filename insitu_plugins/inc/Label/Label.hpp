#ifndef insitu_plugins_Label_HPP
#define insitu_plugins_Label_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>

namespace insitu_plugins
{
class Label : public insitu::Filter
{
public:
    Label(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void) const
    {
        return true;
    }

private:
    void filterInit(void);

    void onDelete(void);

};    // end class Label

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Label_HPP
