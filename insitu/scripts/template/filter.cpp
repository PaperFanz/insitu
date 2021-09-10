#include <{1}/{1}.hpp>
#include <{1}/{1}_dialog.hpp>

namespace {0} {{

/*
    Filter Implementation
*/
{1}::{1}(void)
{{
    // TODO instantiation code
}}

void {1}::filterInit(void)
{{
    settingsDialog = new {1}Dialog(this);
    size = QSize(300, 300);

    // TODO ROS initialization code
}}

void {1}::onDelete(void)
{{
    // TODO cleanup code
}}


const cv::Mat {1}::apply (void)
{{
    /*
        Create a transparent image to construct your overlay on
    */
    cv::Mat ret = cv::Mat(
        height(),
        width(),
        CV_8UC4,
        cv::Scalar(255, 255, 255, 0)
    );

    // TODO edit your overlay from user settings 
    // e.g. settings.get("key", defaultValue).asType()

    return ret;
}}

}} // end namespace {0}

PLUGINLIB_EXPORT_CLASS({0}::{1}, insitu::Filter);

