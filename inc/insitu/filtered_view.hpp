#ifndef insitu_FILTERED_VIEW_HPP
#define insitu_FILTERED_VIEW_HPP

// QT includes
#include <QtWidgets>

// ROS includes
#include <ros/ros.h>
#include <ros/master.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// insitu includes
#include "insitu_utils.hpp"

namespace insitu {

class FilteredView : public QWidget
{

Q_OBJECT
private:
    // UI elements
    QComboBox * topicBox;
    QPushButton * refreshTopicButton;
    QLabel * imgLabel;
    QScrollArea *imgPan;

    // Data
    QString topic;
    QImage img;
    double scale = 1.0;

    // layout elements
    QHBoxLayout * menuBar;
    QVBoxLayout * vBox;

    // ROS
    image_transport::Subscriber sub;
    void onTopicChange(void);
    void callbackImg(const sensor_msgs::Image::ConstPtr& msg);

public Q_SLOTS:
    void refreshTopics(void);

public:
    FilteredView(QString _topic, QWidget * parent = nullptr);

};

} // end namespace insitu

#endif
