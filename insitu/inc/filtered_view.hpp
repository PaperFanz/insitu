#ifndef insitu_FILTERED_VIEW_HPP
#define insitu_FILTERED_VIEW_HPP

// QT includes
#include <QtWidgets>

// ROS includes
#include <ros/ros.h>
#include <ros/master.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// insitu includes
#include "insitu_utils.hpp"
#include <insitu/filter.hpp>

namespace insitu {

class FilteredView : public QWidget
{

Q_OBJECT
private:
    // UI elements
    QComboBox * topicBox;
    QPushButton * refreshTopicButton;
    QPushButton * addFilterButton;
    QListWidget * filterList;
    QFrame * imgFrame;
    QLabel * imgLabel;
    QLabel * fpsLabel;

    // layout element
    QGridLayout * layout;

    // ROS
    ros::Time lastFrameTime;
    image_transport::Subscriber sub;

    // OpenCV
    uint32_t frames;
    cv::Mat imgMat;

    // Filter containers
    std::vector<std::string> filterOrder;
    std::unordered_map<std::string, boost::shared_ptr<insitu::Filter>> filters;

    // Housekeeping
    std::string name;

public Q_SLOTS:

    void onTopicChange(QString topic_transport);

    void refreshTopics(void);

    void openFilterDialog(void);

public:

    FilteredView(QString _name, QString _topic, QWidget * parent = nullptr);

    ~FilteredView(void);

    void addFilter(boost::shared_ptr<insitu::Filter> filter);

private:

    void callbackImg(const sensor_msgs::Image::ConstPtr& msg);

};

} // end namespace insitu

#endif
