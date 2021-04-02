#ifndef insitu_plugins_Manipulation_HPP
#define insitu_plugins_Manipulation_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace insitu_plugins
{
class Manipulation : public insitu::Filter
{
public:
    Manipulation(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void onInit(void);

    void onDelete(void);

    ros::NodeHandle nh_;
    
    tf2_ros::Buffer tfBuffer_;
  
    tf2_ros::TransformListener tfListener_;  

    geometry_msgs::Quaternion quat_msg_;

    geometry_msgs::TransformStamped transformStamped_;
  
    Eigen::Quaterniond quat_eigen;
  
    Eigen::Vector3d  trans_eigen;

};     

}     

#endif  
