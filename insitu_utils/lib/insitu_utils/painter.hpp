#ifndef insitu_utils_PAINTER_HPP
#define insitu_utils_PAINTER_HPP

/* OpenCV includes */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace insitu_utils
{

class Painter {
public:
    static void drawtorect(cv::Mat& mat, cv::Rect target, const std::string& str,
                            int face = cv::FONT_HERSHEY_PLAIN, int thickness = 1,
                            cv::Scalar color = cv::Scalar(255, 255, 255, 255));
};

}

#endif /* insitu_utils_PAINTER_HPP */

