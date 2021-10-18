#ifndef insitu_plugins_Heartbeat_HPP
#define insitu_plugins_Heartbeat_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>
#include "ros_type_introspection/ros_introspection.hpp"
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace insitu_plugins
{
class Heartbeat : public insitu::Filter
{
public:
    Heartbeat(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void) const
    {
        return true;
    }

    void onTopicChange(const std::string& new_topic);

private:
    void filterInit(void);

    void onDelete(void);

    void topicCB(const topic_tools::ShapeShifter::ConstPtr& msg);
    void handleCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                        const std::string& topic_name,
                        RosIntrospection::Parser& parser);

    ros::NodeHandle nh_;

    std::string topic_name_;
    ros::Subscriber topic_subscriber_;

    RosIntrospection::Parser parser_;

    ros::Time last_msg_received_;

};    // end class Heartbeat

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Heartbeat_HPP
