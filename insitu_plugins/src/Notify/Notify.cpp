#include <Notify/Notify.hpp>
#include <Notify/Notify_dialog.hpp>

namespace insitu_plugins
{
/*
    Filter Implementation
*/
Notify::Notify(void)
{
    // TODO instantiation code
}

void Notify::filterInit(void)
{
    settingsDialog = new NotifyDialog(this);
    setSize(QSize(300, 300));

    // TODO ROS initialization code
}

void Notify::onDelete(void)
{
    // TODO cleanup code
}

const cv::Mat Notify::apply(void)
{
    /*
        Create a transparent image to construct your overlay on
    */
    cv::Mat ret =
        cv::Mat(height(), width(), CV_8UC4, cv::Scalar(255, 255, 255, 0));

    // TODO edit your overlay from user settings
    // e.g. settings.get("key", defaultValue).asType()

    return ret;
}

}    // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Notify, insitu::Filter);

