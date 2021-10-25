#ifndef insitu_utils_ROS_HPP
#define insitu_utils_ROS_HPP

#include <vector>
#include <string>

namespace insitu_utils
{
void getTopicsByType(std::vector<std::string>& ret,
                     const std::string& type = "");

}

#endif /* insitu_utils_ROS_HPP */
