#ifndef insitu_plugins_CROSSHAIR_HPP
#define insitu_plugins_CROSSHAIR_HPP

#include <insitu/filter.hpp>
#include <pluginlib/class_list_macros.h>

namespace insitu_plugins {

class Crosshair : public insitu::Filter {
public:
  Crosshair(void);

  virtual cv::Mat apply(cv::Mat);

  virtual bool hasSettingEditor(void) { return true; }

private:
  virtual void onInit(void);

}; // class Text

} // namespace insitu_plugins

#endif // insitu_plugins_CROSSHAIR_HPP
