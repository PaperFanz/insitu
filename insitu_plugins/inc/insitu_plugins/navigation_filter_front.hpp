#ifndef insitu_plugins_NAVIGATION_FILTER_HPP
#define insitu_plugins_NAVIGATION_FILTER_HPP

#include <geometry_msgs/Twist.h>
#include <insitu/filter.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace insitu_plugins {

class NavFilterFront : public insitu::Filter {

public:
  NavFilterFront(void);

  virtual cv::Mat apply(cv::Mat);

  void updateTwistCB(const geometry_msgs::Twist &vel);

  virtual bool hasSettingEditor(void) { return true; }

private:
  ros::NodeHandle nh_;

  ros::Subscriber vel_sub_;

  std::string text;

  geometry_msgs::Twist cur_vel_;

  virtual void onInit(void);
};

} // namespace insitu_plugins

#endif // insitu_plugins_NAVIGATION FILTER_HPP
