#ifndef insitu_plugins_Manip_boundary_HPP
#define insitu_plugins_Manip_boundary_HPP


#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>

namespace insitu_plugins
{
class ManipBoundary : public insitu::Filter
{
public:
    ManipBoundary(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void onInit(void);

    void onDelete(void);

};     

}     

#endif     
