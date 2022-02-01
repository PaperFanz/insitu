#ifndef insitu_plugins_Notification_HPP
#define insitu_plugins_Notification_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <queue>

namespace insitu_plugins
{
class Notification : public insitu::Filter
{
public:
    Notification(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void) const
    {
        return true;
    }

    void onTopicChange(const std::string& new_topic);

    void onQueueChange(const int new_queue_size);

    void onDirectionChange(const bool new_msg_direction_down);

private:
    void filterInit(void);

    void onDelete(void);

    void handleCallback(const std_msgs::String::ConstPtr& msg);

    std::string queueToString(std::queue<std::string> str_queue);

    std::queue<std::string> reverseQueue(std::queue<std::string> str_queue);

    ros::NodeHandle nh_;

    std::string topic_name_;

    ros::Subscriber topic_subscriber_;

    cv::Mat ret_;

    unsigned int queue_size_;

    bool msg_direction_down_;

    std::queue<std::string> msg_queue_;

    std::string msg_string_;

};    // end class Notification

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Notification_HPP
