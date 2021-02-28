#include <insitu_plugins/navigation_filter_front.hpp>

using namespace cv;

namespace insitu_plugins {

NavFilterFront::NavFilterFront(void)

{
  nh_ = ros::NodeHandle();

  vel_sub_ = nh_.subscribe("/husky_velocity_controller/cmd_vel", 1,
                           &NavFilterFront::updateTwistCB, this);

  // ROS_INFO("CONSTRUCTED_Front!");
}

cv::Mat NavFilterFront::apply(cv::Mat img) {

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
  line(img, Point(562, 1185), Point(591, 1086), colorRed, line_thickness);
  line(img, Point(893, 1185), Point(859, 1086), colorRed, line_thickness);
  line(img, Point(593, 1089), Point(619, 1089), colorRed, line_thickness);
  line(img, Point(831, 1089), Point(862, 1089), colorRed, line_thickness);

  line(img, Point(591, 1086), Point(617, 1015), colorYellow, line_thickness);
  line(img, Point(859, 1086), Point(838, 1015), colorYellow, line_thickness);
  line(img, Point(618, 1017), Point(646, 1017), colorYellow, line_thickness);
  line(img, Point(839, 1017), Point(807, 1017), colorYellow, line_thickness);

  line(img, Point(617, 1015), Point(663, 874), colorGreen, line_thickness);
  line(img, Point(838, 1015), Point(790, 874), colorGreen, line_thickness);
  line(img, Point(638, 955), Point(666, 955), colorGreen, line_thickness);
  line(img, Point(819, 955), Point(797, 955), colorGreen, line_thickness);
  line(img, Point(655, 913), Point(673, 913), colorGreen, line_thickness);
  line(img, Point(801, 913), Point(781, 913), colorGreen, line_thickness);

  putText(img, "25 cm", Point(480, 1084), FONT_HERSHEY_COMPLEX, 0.95, colorRed,
          2);
  putText(img, "50 cm", Point(520, 1005), FONT_HERSHEY_COMPLEX, 0.8,
          colorYellow, 2);
  putText(img, "75 cm", Point(538, 950), FONT_HERSHEY_COMPLEX, 0.75, colorGreen,
          2);
  putText(img, "100 cm", Point(545, 910), FONT_HERSHEY_COMPLEX, 0.7, colorGreen,
          2);

  // define static & forward velocity jog lines
  if (cur_vel_.linear.x > 0.01) {
    arrowedLine(img, Point(734, 1092),
                Point(734, 1092 - int(nav_line_scale * cur_vel_.linear.x)),
                colorGreen, line_thickness = 5);
  }

  if (cur_vel_.angular.z < -0.01 && !(cur_vel_.linear.x < -0.01)) {
    ellipse(img, Point(992, 1155),
            Size(100, 200), // half of the size of the ellipse main axes,
            160, 0, 0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness = 5);

    ellipse(img, Point(664, 1185), Size(100, 200), 180, 0,
            0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness = 5);
  }

  if (cur_vel_.angular.z > 0.01 && !(cur_vel_.linear.x < -0.01)) {
    ellipse(img, Point(792, 1187),
            Size(100, 200), // half of the size of the ellipse main axes,
            0, 0, 0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness);

    ellipse(img, Point(464, 1187), Size(100, 200), 0, 0,
            0 - int(nav_line_scale * cur_vel_.angular.z), colorGreen,
            line_thickness);
  }

  // define reverse velocity jog lines
  if (cur_vel_.linear.x < -0.01) {
    arrowedLine(img, Point(734, 1092),
                Point(734, 1092 - int(nav_line_scale * cur_vel_.linear.x)),
                colorGreen, line_thickness = 5);
  }

  return img;
}

void NavFilterFront::updateTwistCB(const geometry_msgs::Twist &vel) {

  cur_vel_ = vel;
  // ROS_INFO("GOT NEW VELOCITY!");
}

} // namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::NavFilterFront, insitu::Filter);
