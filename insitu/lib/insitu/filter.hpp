#ifndef insitu_plugins_FILTER_HPP
#define insitu_plugins_FILTER_HPP

// QT includes
#include <QtWidgets>

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

class Filter;

class FilterDialog : public QDialog
{
Q_OBJECT
protected:

    insitu::Filter * parent;

public:

    FilterDialog(insitu::Filter * parent_)
    {
        parent = parent_;
    }

}; // class FilterDialog

class Filter : public nodelet::Nodelet
{

protected:

    // dialog box for editing settings
    FilterDialog * settingsDialog;

    // settings dictionary; load from ROS param server or configure through GUI
    std::unordered_map<std::string, std::pair<setting_t, std::string>> settings;

    // BEGIN settings getters
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
    // END settings getters

public:

    Filter(){};

    virtual void
    rmFilter(){};

    /*
        @Filter implementors: reimplement this function to apply filter effects;
        this function is called during every image subscriber callback for the
        view it is applied to
    */

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

    /*
        called by Insitu, can also be used to implement custom settings 
        editing interface
    */
    const std::unordered_map<std::string, std::pair<setting_t, std::string>>&
    getSettings(void)
    {
        return settings;
    }

    /*
        called by Insitu, do not reimplement [debug function]
    */
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

    /*
        reimplement if providing a custom settings dialog
    */
    virtual bool
    hasSettingEditor(void)
    {
        return false;
    }

    /*
        called by Insitu, do not reimplement
    */
    virtual void
    openSettingEditor(void)
    {
        settingsDialog->open();
    }

private:

    /*
        @Filter implementors: reimplement this function with any initialization
        required; the creating of data structures, global variables, etc. 
        Called by Insitu upon filter load.
    */
    virtual void
    onInit()
    {
        settingsDialog = new FilterDialog(this);
        settingsDialog->setWindowTitle(QString::fromStdString(name()) + " Settings");
    };

};  // class Filter

}   // namespace insitu

#endif
