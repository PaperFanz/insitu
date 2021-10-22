#include <insitu_utils/painter.hpp>
#include <opencv2/core/types.hpp>

namespace insitu_utils
{

void Painter::drawtorect(cv::Mat& mat, cv::Rect target, const std::string& str,
                         int face, int thickness, cv::Scalar color)
{
    cv::Size rect = cv::getTextSize(str, face, 1.0, thickness, 0);
    double scalex = (double)target.width / (double)rect.width;
    double scaley = (double)target.height / (double)rect.height;
    double scale = std::min(scalex, scaley);
    int marginx =
        scale == scalex ?
            0 :
            (int)((double)target.width * (scalex - scale) / scalex * 0.5);
    int marginy =
        scale == scaley ?
            0 :
            (int)((double)target.height * (scaley - scale) / scaley * 0.5);
    cv::putText(
        mat, str,
        cv::Point(target.x + marginx, target.y + target.height - marginy), face,
        scale, color, thickness, cv::LINE_AA, false);
}

}    // namespace insitu_utils
