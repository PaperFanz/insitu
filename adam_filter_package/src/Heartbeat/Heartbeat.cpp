#include <Heartbeat/Heartbeat.hpp>
#include <Heartbeat/Heartbeat_dialog.hpp>

namespace adam_filter_package
{
/*
    Filter Implementation
*/
Heartbeat::Heartbeat(void)
{
    // TODO instantiation code
}

void Heartbeat::filterInit(void)
{
    settingsDialog = new HeartbeatDialog(this);
    setSize(QSize(300, 300));

    auto nh = getNodeHandle();
    parser_ = RosIntrospection::Parser();

    topic_name_ = "test";

    // boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) >
    // callback; callback = [&parser, topic_name, this](const
    // topic_tools::ShapeShifter::ConstPtr& msg)
    // {
    //     topicCallback(msg, topic_name, parser) ;
    // };
    // ros::Subscriber topic_subscriber_ = nh.subscribe(topic_name_, 1,
    // callback);
    ros::Subscriber topic_subscriber_ =
        nh.subscribe(topic_name_, 1, &Heartbeat::testCB, this);
}

void Heartbeat::onDelete(void)
{
    // TODO cleanup code
}

const cv::Mat Heartbeat::apply(void)
{
    /*
        Create a transparent image to construct your overlay on
    */
    cv::Mat ret =
        cv::Mat(height(), width(), CV_8UC4, cv::Scalar(255, 255, 255, 0));

    cv::Scalar cvColor;
    if ((ros::Time::now() - last_msg_received_).toSec() > 1.0)
    {
        cvColor = cv::Scalar(255, 0, 0, 255);
    }
    else
    {
        cvColor = cv::Scalar(0, 255, 0, 255);
    }

    cv::circle(ret, cv::Point(100, 100), 50, cvColor, -10);

    return ret;
}

void Heartbeat::shapeShifterCB(const topic_tools::ShapeShifter::ConstPtr& msg)
{
    topicCallback(msg, topic_name_, parser_);
}

void Heartbeat::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                              const std::string& topic_name,
                              RosIntrospection::Parser& parser)
{
    last_msg_received_ = ros::Time::now();
    ROS_INFO_STREAM_THROTTLE(1.0, "In callback...");
}

void Heartbeat::testCB(const std_msgs::Bool::ConstPtr& msg)
{
    last_msg_received_ = ros::Time::now();
    ROS_INFO_STREAM_THROTTLE(1.0, "In callback...");
}

}    // end namespace adam_filter_package

PLUGINLIB_EXPORT_CLASS(adam_filter_package::Heartbeat, insitu::Filter);
