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

void Painter::drawtorect_multiline(cv::Mat& mat, cv::Rect target, const std::string& topic_name,
                                   const std::string& str, int lines, int face, int thickness,
                                   cv::Scalar color)
{
    cv::Size rect = cv::getTextSize(topic_name, face, 1.0, thickness, 0);
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
            (int)((double)target.height * (scaley - scale) / scaley * (0.25 * lines));

    /*
        Print topic name header
    */
    cv::putText(
            mat, topic_name,
            cv::Point(target.x + marginx, target.y + marginy), face,
            scale, color, thickness, cv::LINE_AA, false);
    cv::putText(
            mat, "-----",
            cv::Point(target.x + marginx, target.y + marginy * 2), face,
            scale, color, thickness, cv::LINE_AA, false);

    /*
        Print multiple lines of text
    */
    std::string line, text = str;
    int line_height = marginy * 2;
    std::size_t found = str.find_first_of('\n');

    while (found != std::string::npos) {
        line = text.substr(0, found);
        cv::putText(
            mat, line,
            cv::Point(target.x + marginx, target.y + marginy + line_height), face,
            scale, color, thickness, cv::LINE_AA, false);
        
        line_height += marginy;
        text = text.substr(found + 1, std::string::npos);
        found = text.find_first_of('\n');
    }
    cv::putText(
        mat, text,
        cv::Point(target.x + marginx, target.y + marginy + line_height), face,
        scale, color, thickness, cv::LINE_AA, false);
}

}    // namespace insitu_utils
