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

// C++ includes
#include <thread>
#include <chrono>
#include <json/json.h>

using namespace std::chrono_literals;

namespace insitu {

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

private:

    cv::Mat filterBuf;

    std::thread filterThread;

    bool stopped = false;

protected:

    // dialog box for editing settings
    FilterDialog * settingsDialog;

    // settings object
    Json::Value settings;

    void updateFilter(cv::Mat filter)
    {
        filterBuf = filter.clone();
        if (stopped) {
            throw stopped;
        }
    }

public:

    Filter(){};

    virtual void
    rmFilter(){};

    /*
        @Filter implementors: reimplement this function to apply filter effects;
        this function is called during every image subscriber callback for the
        view it is applied to
    */
    virtual void
    run (void)
    {
        cv::Mat m = cv::Mat(
            settings.get("width", 300).asInt(),
            settings.get("height", 300).asInt(),
            CV_8UC4,
            cv::Scalar(255, 255, 255, 0)
        );

        while (true) {
            updateFilter(m);
            std::this_thread::sleep_for(100ms);
        }
    }

    void
    start (void)
    {
        filterThread = std::thread(&Filter::run, this);
    }

    void
    stop (void)
    {
        stopped = true;
        try {
            filterThread.join();
        } catch (bool ret) {
            assert(ret);
        }
    }

    const std::string&
    name(void)
    {
        return getName();
    }

    const cv::Mat&
    getFilterBuf(void)
    {
        return filterBuf;
    }


    /*
        @Filter implementors: reimplement this function to return true if your 
        filter has a custom settings dialog
    */
    bool
    hasSettingEditor(void)
    {
        return false;
    }

    /*
        called by Insitu, do not reimplement
    */
    void
    openSettingEditor(void)
    {
        if (settingsDialog != nullptr) settingsDialog->open();
    }

private:

    /*
        @Filter implementors: reimplement this function with any initialization
        required; the creating of data structures, global variables, etc. All 
        initialization of the ROS infrastructure must be put into this function.
    */
    virtual void
    onInit()
    {
        settingsDialog = new FilterDialog(this);
        settingsDialog->setWindowTitle(QString::fromStdString(name()) + " Settings");

        settings["width"] = 300;
        settings["height"] = 300;
    };

};  // class Filter

}   // namespace insitu

#endif
