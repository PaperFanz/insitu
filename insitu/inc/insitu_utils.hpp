#ifndef insitu_INSITU_UTILS_HPP
#define insitu_INSITU_UTILS_HPP

// QT includes
#include <QtWidgets>
#include <QSet>
#include <QString>

// ROS includes
#include <ros/ros.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>

// C++ includes
#include <vector>
#include <unordered_map>

namespace insitu
{
QList<QString> getModeList(void);

QList<QString> getTopicList(void);

void addNamedWidget(std::string name, QWidget* widget);

QWidget* getNamedWidget(std::string name);

void clearNamedWidgets(void);

void clearLayout(QLayout* layout);

}    // namespace insitu
#endif
