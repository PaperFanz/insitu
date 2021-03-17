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
#include <future>
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

class FilterWatchdog : public QObject
{
Q_OBJECT
private:

    insitu::Filter * parent;
    
public:

    FilterWatchdog(insitu::Filter * parent_)
    {
        parent = parent_;  
    }

    void notify(const cv::Mat& update)
    {
        emit filterUpdated(update);
    }

signals:

    void filterUpdated(const cv::Mat& update);

}; // class FilterWatchdog

class Filter : public nodelet::Nodelet
{

private:

    cv::Mat filterBuf;

    std::promise<void> exitObj;

    std::thread filterThread;

protected:

    // dialog box for editing settings
    FilterDialog * settingsDialog;

    // settings object
    Json::Value settings;

    void updateFilter(const cv::Mat& filter)
    {
        filterBuf = filter.clone();
        filterWatchdog->notify(filterBuf);
    }

public:

    // qobject for notifying InSitu of new filter updates
    FilterWatchdog * filterWatchdog;

    Filter(){};

    /*
        @Filter implementors: reimplement this function to apply filter effects;
        this function MUST CALL updateFilter in its main loop, both to push
        changes in the filter to InSitu and to facilitate filter shutdown
    */
    virtual const cv::Mat
    apply (void)
    {
        cv::Mat ret = cv::Mat(
            settings.get("width", 300).asInt(),
            settings.get("height", 300).asInt(),
            CV_8UC4,
            cv::Scalar(255, 255, 255, 0)
        );

        return ret;
    }

    void
    run (std::future<void> exitCond)
    {
        while (exitCond.wait_for(1ms) == std::future_status::timeout) {
            updateFilter(apply());
            std::this_thread::sleep_for(10ms);
        }
    }

    void
    start (void)
    {
        std::future<void> exitCond = exitObj.get_future();

        filterWatchdog = new FilterWatchdog(this);
        filterThread = std::thread(&Filter::run, this, std::move(exitCond));
    }

    void
    stop (void)
    {
        exitObj.set_value();
        filterThread.join();
        onDelete();
        delete settingsDialog;
        delete filterWatchdog;
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
    virtual bool
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

    /*
        @Filter implementors: reimplement this function to include any cleanup
        operations before filter shutdown
    */
    virtual void
    onDelete(){};

};  // class Filter

}   // namespace insitu

#endif
