#include <Manip_boundary/Manip_boundary.hpp>
#include <Manip_boundary/Manip_boundary_dialog.hpp>

using namespace cv;

namespace insitu_plugins
{
    cv::Scalar colorGreen = cv::Scalar(0, 255, 0);
   
/*
    Filter Implementation
*/
ManipBoundary::ManipBoundary(void)
{
    // TODO instantiation code
}

void ManipBoundary::onInit(void)
{
    settingsDialog = new ManipBoundaryDialog(this);

    // TODO initialization code
}


void ManipBoundary::onDelete(void)
{
    // TODO cleanup code
}


const cv::Mat ManipBoundary::apply(void)
{
    cv::Scalar colorGreen = cv::Scalar(255, 0, 0);

    /*
        Create a transparent image to construct your overlay on
    */
     cv::Mat ret = cv::Mat(settings.get("height", 1440).asInt(),
                          settings.get("width", 1440).asInt(), CV_8UC4,
                                      cv::Scalar(0, 0, 0, 0));
                                      

    cv::Scalar cvColor(0, 255, 0, 255);

    int line_thickness = 5;
    int nav_line_scale = 500;

    //scale lines: 2500 rows, 1360 cols
    double scale_x = 1920.0 / 2500.0;
    double scale_y = 1080.0 / 1300.0;

    int pt_1x = (800 * scale_x);
    int pt_1y = (1337 * scale_y);
    int pt_2x = (1235 * scale_x);
    int pt_2y = (1337 * scale_y);
    int pt_3x = (1310 * scale_x);
    int pt_3y = (800 * scale_y);
    int pt_4x = (818 * scale_x);
    int pt_4y = (510 * scale_y);
    int pt_5x = (320 * scale_x);
    int pt_5y = (522 * scale_y);
    int pt_6x = (325 * scale_x);
    int pt_6y = (808 * scale_y);
    int pt_7x = (576 * scale_x);
    int pt_7y = (927 * scale_y);
    int pt_8x = (800 * scale_x);
    int pt_8y = (1337 * scale_y);

    std::vector<Point> contour;
    int shift_y = 50;
    int shift_x = 400;
    contour.push_back(Point(pt_1x + shift_x, pt_1y - shift_y));
    contour.push_back(Point(pt_2x + shift_x, pt_2y - shift_y));
    contour.push_back(Point(pt_3x + shift_x, pt_3y - shift_y));
    contour.push_back(Point(pt_4x + shift_x, pt_4y - shift_y));
    contour.push_back(Point(pt_5x + shift_x, pt_5y - shift_y));
    contour.push_back(Point(pt_6x + shift_x, pt_6y - shift_y));
    contour.push_back(Point(pt_7x + shift_x, pt_7y - shift_y));
    contour.push_back(Point(pt_8x + shift_x, pt_8y - shift_y));

    const Point *pts = (const cv::Point *)Mat(contour).data;
    int npts = Mat(contour).rows;
    polylines(ret, &pts, &npts, 1, false,cvColor, line_thickness);

    return ret;
}

}     

PLUGINLIB_EXPORT_CLASS(insitu_plugins::ManipBoundary, insitu::Filter);
