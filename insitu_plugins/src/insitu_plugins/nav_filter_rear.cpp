#include <insitu_plugins/crosshair.hpp>
#include <insitu_plugins/crosshair_dialog.hpp>
#include <insitu_plugins/navigation_filter_rear.hpp>

using namespace cv;

namespace insitu_plugins {

NavFilterRear::NavFilterRear(void)

{
  nh_ = ros::NodeHandle();

  vel_sub_ = nh_.subscribe("/husky_velocity_controller/cmd_vel", 1,
                           &NavFilterRear::updateTwistCB, this);

  settings["size"] = {insitu::INT, "5"};
  settings["x"] = {insitu::INT, "0"};
  settings["y"] = {insitu::INT, "0"};

  // ROS_INFO("CONSTRUCTED_rear!");
}

void NavFilterRear::onInit(void) { settingsDialog = new CrosshairDialog(this); }

cv::Mat NavFilterRear::apply(cv::Mat img) {

  int size = getIntSetting("size");
  int x = getIntSetting("x");
  int y = getIntSetting("y");

  // declare colors: RGB from husky_cam
  cv::Scalar colorRed = cv::Scalar(255, 0, 0);
  cv::Scalar colorGreen = cv::Scalar(0, 255, 0);
  cv::Scalar colorYellow = cv::Scalar(255, 255, 0);

  int line_thickness = 5;
  int nav_line_scale = 500; // scale arrow length

  // pixel scaling
  double scale_y = img.cols / 1440;
  double scale_x = img.rows / 1440;

  // option to use perspective eqn to generate points later
  line(img, Point(562 - x, 1185 - y), Point(591 - x, 1086 - y), colorRed,
       line_thickness);
  line(img, Point(893 + x, 1185 - y), Point(859 + x, 1086 - y), colorRed,
       line_thickness);
  line(img, Point(593 - x, 1089 - y), Point(619 - x, 1089 - y), colorRed,
       line_thickness);
  line(img, Point(831 + x, 1089 - y), Point(862 + x, 1089 - y), colorRed,
       line_thickness);

  line(img, Point(591 - x, 1086 - y), Point(617 - x, 1015 - y), colorYellow,
       line_thickness);
  line(img, Point(859 + x, 1086 - y), Point(838 + x, 1015 - y), colorYellow,
       line_thickness);
  line(img, Point(618 - x, 1017 - y), Point(646 - x, 1017 - y), colorYellow,
       line_thickness);
  line(img, Point(839 + x, 1017 - y), Point(807 + x, 1017 - y), colorYellow,
       line_thickness);

  line(img, Point(617 - x, 1015 - y), Point(663 - x, 874 - y), colorGreen,
       line_thickness);
  line(img, Point(838 + x, 1015 - y), Point(790 + x, 874 - y), colorGreen,
       line_thickness);
  line(img, Point(638 - x, 955 - y), Point(666 - x, 955 - y), colorGreen,
       line_thickness);
  line(img, Point(819 + x, 955 - y), Point(797 + x, 955 - y), colorGreen,
       line_thickness);
  line(img, Point(655 - x, 913 - y), Point(673 - x, 913 - y), colorGreen,
       line_thickness);
  line(img, Point(801 + x, 913 - y), Point(781 + x, 913 - y), colorGreen,
       line_thickness);

  putText(img, "25 cm", Point(480 - x, 1084 - y), FONT_HERSHEY_COMPLEX, 0.95,
          colorRed, 2);
  putText(img, "50 cm", Point(520 - x, 1005 - y), FONT_HERSHEY_COMPLEX, 0.8,
          colorYellow, 2);
  putText(img, "75 cm", Point(538 - x, 950 - y), FONT_HERSHEY_COMPLEX, 0.75,
          colorGreen, 2);
  putText(img, "100 cm", Point(545 - x, 910 - y), FONT_HERSHEY_COMPLEX, 0.7,
          colorGreen, 2);

  // velocity jog lines
  if (cur_vel_.linear.x < -0.01 || cur_vel_.linear.x > 0.01) {
    arrowedLine(img, Point(734, 1092),
                Point(734, 1092 + int(nav_line_scale * cur_vel_.linear.x)),
                colorGreen, line_thickness = 5);
  }

  if (cur_vel_.angular.z < 0.01) {
    ellipse(img, Point(992, 1155),
            Size(100, 200), // half of the size of the ellipse main axes,
            160, 0, 0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness = 5);

    ellipse(img, Point(664, 1185), Size(100, 200), 180, 0,
            0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness = 5);
  }

  if (cur_vel_.angular.z > -0.01) {
    ellipse(img, Point(792, 1187),
            Size(100, 200), // half of the size of the ellipse main axes,
            0, 0, 0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness);

    ellipse(img, Point(464, 1187), Size(100, 200), 0, 0,
            0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness);
  }

  return img;
}

void NavFilterRear::updateTwistCB(const geometry_msgs::Twist &vel) {

  cur_vel_ = vel;
  // ROS_INFO("GOT NEW VELOCITY!");
}

} // namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::NavFilterRear, insitu::Filter);
