#include <Notification/Notification.hpp>
#include <Notification/Notification_dialog.hpp>
#include <insitu_utils/painter.hpp>

using namespace notification_filter;

namespace insitu_plugins
{
/*
    Filter Implementation
*/

Notification::Notification(void)
{
    // TODO instantiation code
}

void Notification::filterInit(void)
{
    settingsDialog = new NotificationDialog(this);
    setSize(QSize(300, 300));

    nh_ = getNodeHandle();

    topic_name_ = getSettingsValue().get("topic", DEFAULT_TOPIC).asString();

    topic_subscriber_ =     // 2nd parameter is queue size
        nh_.subscribe(topic_name_, 1, &Notification::handleCallback, this);
    
    queue_size_ = getSettingsValue().get("queue_size", std::stoi(DEFAULT_QUEUE_SIZE)).asInt();

    msg_string_ = queueToString(msg_queue_);

    // TODO ROS initialization code
}

void Notification::onDelete(void)
{
    topic_subscriber_.shutdown();
}


const cv::Mat Notification::apply (void)
{
    /*
        Create a transparent image to construct your overlay on
    */
    ret_ = cv::Mat(
        height(),
        width(),
        CV_8UC4,
        cv::Scalar(255, 255, 255, 0)
    );

    insitu_utils::Painter::drawtorect_multiline(
        ret_, cv::Rect(0, 0, ret_.cols, ret_.rows / (queue_size_ + 2)),
        msg_string_, queue_size_ + 2);

    return ret_;
}

void Notification::handleCallback(const std_msgs::String::ConstPtr& msg)
{
    /*
        Update queue with the most recent messages
    */
    if (msg_queue_.size() >= queue_size_) {
        msg_queue_.pop();
    }
    msg_queue_.push(msg->data);
    msg_string_ = queueToString(msg_queue_);
    // ROS_INFO("%s", msg_string_.c_str());
    
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // msg_queue_.push(msg->data + std::to_string(std::rand() % 5));
    // msg_string_ = queueToString(msg_queue_);
}

std::string Notification::queueToString(std::queue<std::string> str_queue)
{
    std::queue<std::string> q = str_queue;
    std::string str;
    int topic_name_len = topic_name_.length();

    /*
        Add topic name and divider
    */
    str += topic_name_ + "\n";
    for (int i = 0; i < topic_name_len; i++) {
        str += "-";
    }
    str += "\n";

    /*
        Add queue messages
    */
    while (!q.empty()) {
        str += q.front() + "\n";
        q.pop();
    }

    return str;
}

void Notification::onTopicChange(const std::string& new_topic)
{
    try
    {
        topic_name_ = new_topic;
        topic_subscriber_ =     // 2nd parameter is queue size
            nh_.subscribe(topic_name_, 1, &Notification::handleCallback, this);
    }
    catch (ros::Exception& e) {
        ROS_ERROR("Error occured: %s ", e.what());
    }
}

void Notification::onQueueChange(const int new_queue_size)
{
    /*
        Update queue size
    */
    queue_size_ = new_queue_size;

    /*
        Clear queue
    */
    std::queue<std::string>().swap(msg_queue_);
    msg_string_ = queueToString(msg_queue_);
}


} // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Notification, insitu::Filter);
