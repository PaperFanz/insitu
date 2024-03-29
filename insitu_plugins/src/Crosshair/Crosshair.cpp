#include <Crosshair/Crosshair.hpp>
#include <Crosshair/Crosshair_dialog.hpp>

namespace insitu_plugins
{
/*
    Filter Implementation
*/
Crosshair::Crosshair(void)
{
    // TODO instantiation code
}

void Crosshair::filterInit(void)
{
    settingsDialog = new CrosshairDialog(this);

    setSize(QSize(300, 300));
}

void Crosshair::onDelete(void)
{
    // TODO cleanup code
}

const cv::Mat Crosshair::apply(void)
{
    cv::Mat ret = cv::Mat(width(), height(), CV_8UC4, cv::Scalar(0, 0, 0, 0));
    color = (color + 1) % 256;

    cv::Scalar cvColor(color, color, color, 255);

    cv::line(ret, cv::Point(150, 100), cv::Point(150, 200), cvColor, 5);
    cv::line(ret, cv::Point(100, 150), cv::Point(200, 150), cvColor, 5);

    return ret;
}

}    // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Crosshair, insitu::Filter);
