#include "insitu_utils.hpp"

namespace insitu
{
const QString IT_PREFIX = "image_transport/";

QList<QString> getTopicList()
{
    ros::NodeHandle nh;

    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");
    QSet<QString> message_sub_types;
    message_sub_types.insert("sensor_msgs/CompressedImage");

    // get declared transports
    QList<QString> transports;
    image_transport::ImageTransport img_tpt(nh);
    std::vector<std::string> declared = img_tpt.getDeclaredTransports();
    for (auto it = declared.begin(); it != declared.end(); it++)
    {
        // qDebug("ImageView::updateTopicList() declared transport '%s'",
        // it->c_str());
        QString transport = it->c_str();

        // strip prefix from transport name
        if (transport.startsWith(IT_PREFIX))
        {
            transport = transport.mid(IT_PREFIX.length());
        }
        transports.append(transport);
    }

    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    QSet<QString> all_topics;
    for (auto it = topic_info.begin(); it != topic_info.end(); it++)
    {
        all_topics.insert(it->name.c_str());
    }

    QSet<QString> topics;
    for (auto it = topic_info.begin(); it != topic_info.end(); it++)
    {
        if (message_types.contains(it->datatype.c_str()))
        {
            QString topic = it->name.c_str();

            // add raw topic
            topics.insert(topic);
            // qDebug("ImageView::getTopics() raw topic '%s'",
            // topic.toStdString().c_str());

            // add transport specific sub-topics
            for (auto jt = transports.begin(); jt != transports.end(); jt++)
            {
                if (all_topics.contains(topic + " " + *jt))
                {
                    QString sub = topic + " " + *jt;
                    topics.insert(sub);
                    // qDebug("ImageView::getTopics() transport specific
                    // sub-topic '%s'", sub.toStdString().c_str());
                }
            }
        }
        if (message_sub_types.contains(it->datatype.c_str()))
        {
            QString topic = it->name.c_str();
            int index = topic.lastIndexOf("/");
            if (index != -1)
            {
                topic.replace(index, 1, " ");
                topics.insert(topic);
                // qDebug("ImageView::getTopics() transport specific sub-topic
                // '%s'", topic.toStdString().c_str());
            }
        }
    }

    return topics.values();
}

static std::unordered_map<std::string, QWidget*> widgetMap;

void addNamedWidget(std::string name, QWidget* widget)
{
    widgetMap[name] = widget;
}

QWidget* getNamedWidget(std::string name)
{
    auto search = widgetMap.find(name);
    if (search != widgetMap.end())
    {
        return search->second;
    }
    else
    {
        return Q_NULLPTR;
    }
}

void clearLayout(QLayout* layout)
{
    while (QLayoutItem* item = layout->takeAt(0))
    {
        if (QWidget* widget = item->widget()) widget->deleteLater();

        if (QLayout* childLayout = item->layout()) clearLayout(childLayout);

        delete item;
    }
}

}    // namespace insitu
