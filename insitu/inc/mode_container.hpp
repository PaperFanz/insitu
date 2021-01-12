#ifndef insitu_MODE_CONTAINER_HPP
#define insitu_MODE_CONTAINER_HPP

// QT includes
#include <QtWidgets>

// ROS includes
#include <ros/ros.h>

// insitu includes
#include "insitu_utils.hpp"
#include "filtered_view.hpp"

namespace insitu {

class ModeContainer : public QWidget
{

Q_OBJECT
private:
    // UI elements
    QPushButton * addViewButton;
    QPushButton * tileButton;
    QPushButton * cascadeButton;
    QMdiArea * container;

    // layout element
    QGridLayout * layout;

    // ROS
    ros::NodeHandle * nh;

public Q_SLOTS:
    void tile(void);
    void cascade(void);

public:
    ModeContainer(QString name, QWidget * parent = nullptr);

    ~ModeContainer(void);

    void addView(FilteredView * view);

    const ros::NodeHandle& getNodeHandle(void);

};

} // end namespace insitu

#endif
