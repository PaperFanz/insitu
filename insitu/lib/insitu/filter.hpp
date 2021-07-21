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
#include <qsize.h>

using namespace std::chrono_literals;

namespace insitu
{
class Filter;

class FilterDialog : public QDialog
{
    Q_OBJECT
protected:
    insitu::Filter* parent;

public:
    FilterDialog(insitu::Filter* parent_)
    {
        parent = parent_;
    }

};    // class FilterDialog

class FilterWatchdog : public QObject
{
    Q_OBJECT
private:
    QGraphicsItem* graphicsItem;

public:
    FilterWatchdog(QGraphicsItem* item)
    {
        graphicsItem = item;
    }

    void notify(const cv::Mat& update)
    {
        emit filterUpdated(graphicsItem, update);
    }

    QGraphicsItem* getGraphicsItem(void) const
    {
        return graphicsItem;
    }

signals:

    void filterUpdated(QGraphicsItem* item, const cv::Mat& update);

};    // class FilterWatchdog

class Filter : public nodelet::Nodelet
{
private:
    cv::Mat filterBuf;

    std::promise<void> exitObj;

    std::thread filterThread;

    FilterWatchdog* filterWatchdog;

    QSize size;

protected:
    // dialog box for editing settings
    FilterDialog* settingsDialog;

    // settings object
    Json::Value settings;

    void updateFilter(const cv::Mat& filter)
    {
        filterBuf = filter.clone();
        filterWatchdog->notify(filterBuf);
    }

public:
    Filter(void){};

    /*
        @Filter implementors: reimplement this function to apply filter effects
    */
    virtual const cv::Mat apply(void)
    {
        cv::Mat ret = cv::Mat(width(),
                              height(), CV_8UC4,
                              cv::Scalar(255, 255, 255, 0));

        return ret;
    }

    /*
        @Filter implementors: reimplement this function to return true if your
        filter has a custom settings dialog
    */
    virtual bool hasSettingEditor(void)
    {
        return false;
    }

    /*
        @Filter implementors: advanced filters may be limited by the interface
        provided by the apply() function, therefore this run() function is left
        virtual to allow users to implement their own threads. However, users
        MUST wait on exitCond and call updateFilter(), or InSitu will not be
        able to render or unload the filter.
    */
    virtual void run(std::future<void> exitCond)
    {
        while (exitCond.wait_for(1ms) == std::future_status::timeout)
        {
            updateFilter(apply());
            std::this_thread::sleep_for(10ms);
        }
    }

    /*
        called by Insitu
    */
    void start(QGraphicsItem* item)
    {
        std::future<void> exitCond = exitObj.get_future();
        filterWatchdog = new FilterWatchdog(item);
        filterThread = std::thread(&Filter::run, this, std::move(exitCond));
    }

    void stop(void)
    {
        exitObj.set_value();
        filterThread.join();
        onDelete();
        delete settingsDialog;
    }

    Json::Value& getSettingsValue(void)
    {
        return settings;
    }

    const std::string& name(void) const
    {
        return getName();
    }

    QSize getSize(void) const
    {
        return size;
    }

    int width(void) const
    {
        return size.width();
    }

    int height(void) const
    {
        return size.height();
    }

    void setSize(QSize size)
    {
        this->size = size;
    }

    void setWidth(int w)
    {
        size.setWidth(w);
    }

    void setHeight(int h)
    {
        size.setHeight(h);
    }

    void openSettingEditor(void)
    {
        if (settingsDialog != nullptr) settingsDialog->open();
    }

    FilterWatchdog* getFilterWatchDog(void) const
    {
        return filterWatchdog;
    }

    QGraphicsItem* getGraphicsItem(void) const
    {
        return filterWatchdog->getGraphicsItem();
    }

protected:
    /*
        @Filter implementors: reimplement this function with any initialization
        required; the creating of data structures, global variables, etc. All
        initialization of the ROS infrastructure must be put into this function.
    */
    virtual void onInit(void)
    {
        settingsDialog = new FilterDialog(this);
        settingsDialog->setWindowTitle(QString::fromStdString(name()) +
                                       " Settings");
        size = QSize(300,300);
    };

    /*
        @Filter implementors: reimplement this function to include any cleanup
        operations before filter shutdown
    */
    virtual void onDelete(void){};

};    // class Filter

}    // namespace insitu

#endif
