#ifndef insitu_INSITU_UTILS_HPP
#define insitu_INSITU_UTILS_HPP

// QT includes
#include <QSet>
#include <QStringList>

// ROS includes
#include <ros/ros.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>

// C++ includes
#include <vector>

namespace insitu {

QList<QString> getModeList();

QList<QString> getTopicList();

} // namespace insitu
#endif