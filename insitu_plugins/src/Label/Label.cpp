#include <Label/Label.hpp>
#include <Label/Label_dialog.hpp>

namespace insitu_plugins {

/*
    Filter Implementation
*/
Label::Label(void)
{
    // TODO instantiation code
}

void Label::onInit(void)
{
    settingsDialog = new LabelDialog(this);

    // TODO initialization code
}

void Label::onDelete(void)
{
    // TODO cleanup code
}

void drawtorect(cv::Mat & mat, cv::Rect target, const std::string & str,
    int face = cv::FONT_HERSHEY_PLAIN, 
    int thickness = 1, 
    cv::Scalar color = cv::Scalar(255, 255, 255, 255))
{
    cv::Size rect = cv::getTextSize(str, face, 1.0, thickness, 0);
    double scalex = (double)target.width / (double)rect.width;
    double scaley = (double)target.height / (double)rect.height;
    double scale = std::min(scalex, scaley);
    int marginx = scale == scalex ? 0 : (int)((double)target.width * (scalex - scale) / scalex * 0.5);
    int marginy = scale == scaley ? 0 : (int)((double)target.height * (scaley - scale) / scaley * 0.5);
    cv::putText(mat, str, cv::Point(target.x + marginx, target.y + target.height - marginy), face, scale, color, thickness, 8, false);
}

const cv::Mat Label::apply (void)
{
    cv::Mat ret = cv::Mat(
        settings.get("height", 100).asInt(),
        settings.get("width", 300).asInt(),
        CV_8UC4,
        cv::Scalar(255, 255, 255, 0)
    );

    drawtorect(ret, cv::Rect(0, 0, ret.cols, ret.rows), 
        settings.get("text", "default").asString());

    return ret;
}

} // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Label, insitu::Filter);
