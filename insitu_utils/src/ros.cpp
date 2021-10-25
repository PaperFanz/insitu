#include "insitu_utils/ros.hpp"
#include <ros/master.h>

namespace insitu_utils
{
void getTopicsByType(std::vector<std::string>& ret, const std::string& type)
{
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    ret.clear();
    for (const auto& topic : topic_infos)
    {
        if (topic.datatype == type || type == "")
        {
            ret.push_back(topic.name);
        }
    }
}

}    // namespace insitu_utils
