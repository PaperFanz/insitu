#ifndef insitu_plugins_TEXT_FILTER_HPP
#define insitu_plugins_TEXT_FILTER_HPP

#include <insitu/filter.hpp>
#include <pluginlib/class_list_macros.h>

namespace insitu_plugins {

class TextFilter : public insitu::Filter {

public:
  TextFilter(void);

  virtual cv::Mat apply(cv::Mat);

private:
  std::string text;

}; // class Text

} // namespace insitu_plugins

#endif // insitu_plugins_TEXT_FILTER_HPP
