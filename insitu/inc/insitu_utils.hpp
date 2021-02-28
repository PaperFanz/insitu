#ifndef insitu_INSITU_UTILS_HPP
#define insitu_INSITU_UTILS_HPP

// QT includes
#include <QSet>
#include <QString>
#include <QtWidgets>

// ROS includes
#include <image_transport/image_transport.h>
#include <ros/master.h>
#include <ros/ros.h>

// C++ includes
#include <unordered_map>
#include <vector>

namespace insitu {

QList<QString> getModeList();

QList<QString> getTopicList();

void addNamedWidget(std::string name, QWidget *widget);

QWidget *getNamedWidget(std::string name);

void clearLayout(QLayout *layout);

} // namespace insitu
#endif