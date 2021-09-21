#ifndef adam_filter_package_Heartbeat_HPP
#define adam_filter_package_Heartbeat_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>
#include "ros_type_introspection/ros_introspection.hpp"
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "std_msgs/Bool.h"

namespace adam_filter_package
{
class Heartbeat : public insitu::Filter
{
public:
    Heartbeat(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void filterInit(void);

    void onDelete(void);

    void topicCB(const topic_tools::ShapeShifter::ConstPtr& msg);
    void handleCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                        const std::string& topic_name,
                        RosIntrospection::Parser& parser);

    std::string topic_name_;
    ros::Subscriber topic_subscriber_;

    RosIntrospection::Parser parser_;

    ros::Time last_msg_received_ = ros::Time(0.);

};    // end class Heartbeat

}    // end namespace adam_filter_package

#endif    // end adam_filter_package_Heartbeat_HPP
