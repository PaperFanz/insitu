#include <insitu_plugins/crosshair.hpp>
#include <insitu_plugins/crosshair_dialog.hpp>

namespace insitu_plugins {

/*
    Filter Implementation
*/
Crosshair::Crosshair(void)
{
    settings["size"] = {insitu::INT, "5"};
    settings["x"] = {insitu::INT, "320"};
    settings["y"] = {insitu::INT, "240"};
}

void Crosshair::onInit(void)
{
    settingsDialog = new CrosshairDialog(this);
}

cv::Mat Crosshair::apply(cv::Mat img)
{
    int size = getIntSetting("size");
    int x = getIntSetting("x");
    int y = getIntSetting("y");
    cv::line(
        img,
        cv::Point(x, y + size),
        cv::Point(x, y - size),
        cv::Scalar(255,0,0)
    );
    cv::line(
        img,
        cv::Point(x + size, y),
        cv::Point(x - size, y),
        cv::Scalar(255,0,0)
    );
    return img;
}

} // namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Crosshair, insitu::Filter);
