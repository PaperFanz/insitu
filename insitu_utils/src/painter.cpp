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

void Painter::drawtorect_multiline(cv::Mat& mat, cv::Rect target, const std::string& str,
                                   int num_lines, int face, int thickness, cv::Scalar color)
{
    double scaley = (double)target.height / num_lines;
    double scale = scaley * 0.5 / 10;           // Letter height is smaller to leave whitespace between lines
    int marginx = num_lines / 3;                // Prevent first letter from getting cut off
    int marginy = scaley;

    /*
        Print multiple lines of text
    */
    std::string text = str;
    std::string line;
    int line_height = marginy * 0.7;
    std::size_t found = text.find_first_of('\n');

    while (found != std::string::npos) {        // Loop to print every line
        line = text.substr(0, found);
        cv::putText(
            mat, line,
            cv::Point(target.x + marginx, target.y + line_height), face,
            scale, color, thickness, cv::LINE_AA, false);
        
        line_height += marginy;    // Position for next line
        text = text.substr(found + 1, std::string::npos);
        found = text.find_first_of('\n');
    }
    cv::putText(
        mat, text,
        cv::Point(target.x + marginx, target.y + marginy + line_height), face,
        scale, color, thickness, cv::LINE_AA, false);
}

}    // namespace insitu_utils
