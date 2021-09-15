#ifndef insitu_MODE_CONTAINER_HPP
#define insitu_MODE_CONTAINER_HPP

// QT includes
#include <QtWidgets>

// ROS includes
#include <ros/ros.h>

// insitu includes
#include "insitu_utils.hpp"
#include "filtered_view.hpp"

// C++ includes
#include <json/json.h>

namespace insitu
{
class ModeContainer : public QWidget
{
    Q_OBJECT
private:
    // UI elements
    QPushButton* addViewButton;
    QPushButton* tileButton;
    QPushButton* cascadeButton;
    QMdiArea* container;

    // layout element
    QGridLayout* layout;

    // ROS
    ros::NodeHandle* nh;

    /* data */
    std::string name;

public Q_SLOTS:
    void tile(void);

    void cascade(void);

    void openViewDialog(void);

public:
    ModeContainer(QString _name, QWidget* parent = nullptr);

    ModeContainer(const Json::Value& json, QWidget* parent = nullptr);

    ~ModeContainer(void);

    void addView(FilteredView* view);

    const ros::NodeHandle& getNodeHandle(void);

    void save(Json::Value& json) const;

    void restore(const Json::Value& json);
};

}    // end namespace insitu

#endif

