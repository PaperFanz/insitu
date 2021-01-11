#ifndef insitu_plugins_FILTER_HPP
#define insitu_plugins_FILTER_HPP

// ros/pluginlib includes
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <unordered_map>

namespace insitu {

typedef enum SettingType {
    BOOLEAN,
    FLOAT,
    INT,
    STRING,
    NUM_SETTING_TYPES
} setting_t;

class Filter : public nodelet::Nodelet
{

public:

    Filter(){};

    virtual void rmFilter(){};

    virtual cv::Mat apply(cv::Mat img){return img;};

    const std::string& name(void){return getName();}

private:

    virtual void onInit(){};

    std::unordered_map<std::string, std::pair<setting_t, std::string>> settings;

};  // class Filter

}   // namespace insitu

#endif
