#include <Label/Label.hpp>
#include <Label/Label_dialog.hpp>
#include <insitu_utils/painter.hpp>

namespace insitu_plugins
{
/*
    Filter Implementation
*/
Label::Label(void)
{
    // TODO instantiation code
}

void Label::filterInit(void)
{
    settingsDialog = new LabelDialog(this);
    setSize(QSize(300, 100));

    // TODO initialization code
}

void Label::onDelete(void)
{
    // TODO cleanup code
}

const cv::Mat Label::apply(void)
{
    cv::Mat ret =
        cv::Mat(height(), width(), CV_8UC4, cv::Scalar(255, 255, 255, 0));

    insitu_utils::Painter::drawtorect(
        ret, cv::Rect(0, 0, ret.cols, ret.rows),
        getSettingsValue().get("text", "text not set").asString());

    return ret;
}

}    // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Label, insitu::Filter);
