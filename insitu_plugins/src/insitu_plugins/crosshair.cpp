#include <insitu_plugins/crosshair.hpp>

namespace insitu_plugins {

Crosshair::Crosshair(void)
{
    settings["size"] = {insitu::INT, "5"};
}

cv::Mat Crosshair::apply(cv::Mat img)
{
    int size = getIntSetting("size");
    cv::line(
        img,
        cv::Point(img.cols/2, img.rows/2 + size),
        cv::Point(img.cols/2, img.rows/2 - size),
        cv::Scalar(0,255,0)
    );
    cv::line(
        img,
        cv::Point(img.cols/2 + size, img.rows/2),
        cv::Point(img.cols/2 - size, img.rows/2),
        cv::Scalar(0,255,0)
    );
    return img;
}

} // namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Crosshair, insitu::Filter);
