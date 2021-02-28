#ifndef insitu_FILTERED_VIEW_HPP
#define insitu_FILTERED_VIEW_HPP

// QT includes
#include <QtWidgets>

// ROS includes
#include <image_transport/image_transport.h>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// insitu includes
#include "insitu_utils.hpp"
#include "ros_image_frame.hpp"
#include <insitu/filter.hpp>

namespace insitu {

class FilteredView : public QWidget {

  Q_OBJECT
private:
  // UI elements
  QComboBox *topicBox;
  QPushButton *refreshTopicButton;
  QPushButton *addFilterButton;
  QPushButton *rmFilterButton;
  QListWidget *filterList;
  RosImageFrame *imgFrame;
  QLabel *imgLabel;
  QLabel *fpsLabel;

  // layout element
  QGridLayout *layout;

  // ROS
  ros::NodeHandle *nh;
  /* Give each view its own callback queue
     and spinner thread to speed things up */
  ros::CallbackQueue viewQueue;
  ros::AsyncSpinner *spinner;
  ros::Time lastFrameTime;
  image_transport::Subscriber sub;

  // OpenCV
  uint32_t frames;
  cv::Mat imgMat;
  QImage imgbuf;

  // Filter container
  std::unordered_map<std::string, boost::shared_ptr<insitu::Filter>> filters;

  // Housekeeping
  std::string name;

public Q_SLOTS:

  void onTopicChange(QString topic_transport);

  void refreshTopics(void);

  void openFilterDialog(void);

  void rmFilter(void);

public:
  FilteredView(const ros::NodeHandle &parent_, QString _name, QString _topic,
               QWidget *parent = nullptr);

  ~FilteredView(void);

  void addFilter(boost::shared_ptr<insitu::Filter> filter);

  const ros::NodeHandle &getNodeHandle(void);

private:
  void callbackImg(const sensor_msgs::Image::ConstPtr &msg);
};

} // end namespace insitu

#endif
