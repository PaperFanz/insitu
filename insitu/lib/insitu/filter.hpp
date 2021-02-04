#ifndef insitu_plugins_FILTER_HPP
#define insitu_plugins_FILTER_HPP

// ros/pluginlib includes
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <unordered_map>
#include <mutex>
#include <boost/lexical_cast.hpp>

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

protected:

    std::unordered_map<std::string, std::pair<setting_t, std::string>> settings;

    // not needed if filter doesn't also write to settings
    // std::mutex settings_mutex;
    
    bool
    getBoolSetting(std::string key)
    {
        // settings_mutex.lock();
        bool b = boost::lexical_cast<bool>(settings[key].second);
        // settings_mutex.unlock();
        return b;
    }

    float
    getFloatSetting(std::string key)
    {
        // settings_mutex.lock();
        float f = boost::lexical_cast<float>(settings[key].second);
        // settings_mutex.unlock();
        return f;
    }

    int
    getIntSetting(std::string key)
    {
        // settings_mutex.lock();
        int i = boost::lexical_cast<int>(settings[key].second);
        // settings_mutex.unlock();
        return i;
    }

    std::string
    getStringSetting(std::string key)
    {
        // settings_mutex.lock();
        std::string s = settings[key].second;
        // settings_mutex.unlock();
        return s;
    }

public:

    Filter(){};

    virtual void
    rmFilter(){};

    virtual cv::Mat
    apply(cv::Mat img)
    {
        return img;
    };

    const std::string&
    name(void)
    {
        return getName();
    }

    const std::unordered_map<std::string, std::pair<setting_t, std::string>>&
    getSettings(void)
    {
        return settings;
    }

    bool
    set(std::string key, std::string val)
    {
        auto it = settings.find(key);
        if (it != settings.end()) {
            it->second.second = val;
            return true;
        } else {
            return false;
        }
    }

private:

    virtual void
    onInit(){};

};  // class Filter

}   // namespace insitu

#endif
