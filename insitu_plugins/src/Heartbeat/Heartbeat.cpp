#include <Heartbeat/Heartbeat.hpp>
#include <Heartbeat/Heartbeat_dialog.hpp>
#include <insitu_utils/painter.hpp>

using namespace heartbeat_filter;

namespace insitu_plugins
{
/*
    Filter Implementation
*/
Heartbeat::Heartbeat(void) : last_msg_received_(ros::Time::now())
{
    // TODO instantiation code
}

void Heartbeat::filterInit(void)
{
    settingsDialog = new HeartbeatDialog(this);
    setSize(QSize(50, 50));

    nh_ = getNodeHandle();
    parser_ = RosIntrospection::Parser();

    topic_name_ = getSettingsValue().get("topic", DEFAULT_TOPIC).asString();

    topic_subscriber_ =
        nh_.subscribe(topic_name_, 1, &Heartbeat::topicCB, this);
}

void Heartbeat::onDelete(void)
{
    // TODO cleanup code
}

const cv::Mat Heartbeat::apply(void)
{
    cv::Mat ret =
        cv::Mat(height(), width(), CV_8UC4, cv::Scalar(255, 255, 255, 0));

    double expected_hz =
        getSettingsValue().get("rate", std::stod(DEFAULT_RATE)).asDouble();

    double time_since_last_received =
        (ros::Time::now() - last_msg_received_).toSec();
    int grad = std::max(0.0, time_since_last_received * expected_hz * 255.0);
    cv::Scalar cvColor(2 * grad, 2 * (255 - grad), 0, 255);

    int radius = std::min(width(), height()) / 2;
    cv::circle(ret, cv::Point(width() / 2, height() / 2), radius, cvColor, -10);

    insitu_utils::Painter::drawtorect(
        ret, cv::Rect(0, 0, ret.cols, ret.rows / 5),
        getSettingsValue().get("name", DEFAULT_NAME).asString());

    return ret;
}

void Heartbeat::topicCB(const topic_tools::ShapeShifter::ConstPtr& msg)
{
    handleCallback(msg, topic_name_, parser_);
}

void Heartbeat::handleCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                               const std::string& topic_name,
                               RosIntrospection::Parser& parser)
{
    last_msg_received_ = ros::Time::now();
}

void Heartbeat::onTopicChange(const std::string& new_topic)
{
    try
    {
        topic_name_ = new_topic;
        topic_subscriber_ =
            nh_.subscribe(topic_name_, 1, &Heartbeat::topicCB, this);
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Error occured: %s ", e.what());
    }
}
}    // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Heartbeat, insitu::Filter);
