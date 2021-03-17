#ifndef insitu_FILTERED_VIEW_HPP
#define insitu_FILTERED_VIEW_HPP

// QT includes
#include <QtWidgets>

// ROS includes
#include <ros/ros.h>
#include <ros/master.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// insitu includes
#include "insitu_utils.hpp"
#include <insitu/filter.hpp>
#include "ros_image_frame.hpp"
#include "filter_graphics_scene.hpp"

namespace insitu {

class FilteredView : public QWidget
{

Q_OBJECT
private:
    // UI elements
    QComboBox * topicBox;
    QPushButton * refreshTopicButton;
    QPushButton * addFilterButton;
    QPushButton * rmFilterButton;
    QCheckBox * republishCheckBox;
    QCheckBox * showFilterPaneCheckBox;
    QListWidget * filterList;
    QGraphicsView * filterView;
    FilterGraphicsScene * filterScene;
    FilterGraphicsItem * rosImg;
    QLabel * fpsLabel;
    QErrorMessage * errMsg;

    // layout elements
    QWidget * filterPaneWidget;
    QGridLayout * filterPaneLayout;
    QHBoxLayout * imagePane;
    QGridLayout * layout;

    // ROS
    ros::NodeHandle * nh;
    /* Give each view its own callback queue 
       and spinner thread to speed things up */
    ros::CallbackQueue viewQueue;
    ros::AsyncSpinner * spinner;
    ros::Time lastFrameTime;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;

    // OpenCV
    bool firstFrame = true;
    uint32_t frames;
    cv::Mat imgMat;
    QImage imgbuf;

    // Filter structs
    std::unordered_map<std::string, boost::shared_ptr<insitu::Filter>> filters;

    // Housekeeping
    std::string name;

public Q_SLOTS:

    void onTopicChange(QString topic_transport);

    void refreshTopics(void);

    void openFilterDialog(void);

    void rmFilter(void);

    void onToggleFilterPane(void);

    void onToggleRepublish(void);

public:

    FilteredView(const ros::NodeHandle& parent_, QString _name, QString _topic, 
        QWidget * parent = nullptr);

    ~FilteredView(void);

    void addFilter(boost::shared_ptr<insitu::Filter> filter);

    const std::string & getViewName(void);

    const ros::NodeHandle & getNodeHandle(void);

    void applyFilters(void);

private:

    void callbackImg(const sensor_msgs::Image::ConstPtr& msg);

};

} // end namespace insitu

#endif
