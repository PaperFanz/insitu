#ifndef insitu_plugins_FILTER_HPP
#define insitu_plugins_FILTER_HPP

// ros/pluginlib includes
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace insitu {

enum SettingType {
    BOOLEAN,
    FLOAT,
    INT,
    STRING,
    NUM_SETTING_TYPES
};

class Filter : public nodelet::Nodelet
{

public:
    Filter(){};
    virtual void rmFilter(){};
    virtual cv::Mat apply(cv::Mat img){return img;};

private:
    virtual void onInit(){};

};  // class Filter

}   // namespace insitu

#endif
