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
#include <insitu/filter.hpp>
#include "filter_factory.hpp"
#include "filter_graphics_view.hpp"
#include "filter_properties.hpp"

// C++ includes
#include <json/json.h>

namespace insitu
{
class FilteredView : public QWidget
{
    Q_OBJECT
private:
    // UI elements
    
    /* top bar */
    QComboBox* topicBox;
    QPushButton* refreshTopicButton;
    QPushButton* addFilterButton;
    QPushButton* rmFilterButton;
    QCheckBox* republishCheckBox;
    QCheckBox* showFilterPaneCheckBox;
    
    /* side bar */
    QListWidget* filterList;
    FilterProperties* filterProps;
    
    /* main view */
    FilterGraphicsView* filterView;
    QGraphicsScene* filterScene;
    FilterGraphicsItem* rosImg;
    
    /* bottom bar */
    QLabel* fpsLabel;
    QErrorMessage* errMsg;

    // layout elements
    QSplitter* filterPaneSplitter;
    QWidget* filterPaneWidget;
    QGridLayout* filterPaneLayout;
    QHBoxLayout* imagePane;
    QGridLayout* layout;

    // ROS
    ros::NodeHandle* nh;
    /* Give each view its own callback queue
       and spinner thread to speed things up */
    ros::CallbackQueue viewQueue;
    ros::AsyncSpinner* spinner;
    ros::Time lastFrameTime;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    bool topicChanged;

    // OpenCV
    uint32_t frames;
    cv_bridge::CvImageConstPtr cv_ptr;
    QImage filteredImg;

    // Filter structs
    FilterFactory* filterFactory;
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

    void onFilterOrderChanged(void);

    void updateFilter(QGraphicsItem* item, const cv::Mat& update);

public:
    FilteredView(const ros::NodeHandle& parent_, QString _name, QString _topic,
                 QWidget* parent = nullptr);

    FilteredView(const ros::NodeHandle& parent_, const Json::Value& json, QWidget* parent = nullptr);

    ~FilteredView(void);

    std::string getViewTopic(void) const;

    void addFilter(boost::shared_ptr<insitu::Filter> filter);

    const std::string& getViewName(void) const;

    const ros::NodeHandle& getNodeHandle(void) const;

    void save(Json::Value& json) const;

    void restore(const Json::Value& json);

private:
    void callbackImg(const sensor_msgs::Image::ConstPtr& msg);

    void unloadFilter(QListWidgetItem* filterItem);
};

}    // end namespace insitu

#endif
