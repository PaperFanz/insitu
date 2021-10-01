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
#include <string>
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

    std::string baseImageTopic;

public:
    void notify(const cv::Mat& update)
    {
        emit filterUpdated(graphicsItem, update);
    }

    void setImageTopic(const std::string& topic)
    {
        baseImageTopic = topic;
    }

    const std::string& imageTopic(void) const
    {
        return baseImageTopic;
    }

    void setGraphicsItem(QGraphicsItem* item)
    {
        graphicsItem = item;
    }

    QGraphicsItem* getGraphicsItem(void) const
    {
        return graphicsItem;
    }

signals:

    void filterUpdated(QGraphicsItem* item, const cv::Mat& update);

public Q_SLOTS:

    void onTopicChanged(const QString& topic)
    {
        baseImageTopic = topic.toStdString();
    }

};    // class FilterWatchdog

typedef enum FilterProps
{
    setToImageSize = 1,
    keepAspectRatio = 2,
    lockFilterProperties = 4
} FilterProps;

class Filter : public nodelet::Nodelet
{
private:
    /* implement async updates */
    std::promise<void> exitObj;
    std::thread filterThread;
    FilterWatchdog filterWatchdog;

    /* track filter properties */
    QSize size;
    int props;

    /* bookkeeping */
    cv::Mat filterBuf;
    std::string type;

protected:
    // dialog box for editing settings
    FilterDialog* settingsDialog;

    // settings object
    Json::Value settings;

    void updateFilter(const cv::Mat& filter)
    {
        filterBuf = filter.clone();
        filterWatchdog.notify(filterBuf);
    }

public:
    /*
        @Filter implementors: reimplement this function to apply filter effects
    */
    virtual const cv::Mat apply(void) = 0;

    /*
        @Filter implementors: reimplement this function to return true if your
        filter has a custom settings dialog
    */
    virtual bool hasSettingEditor(void)
    {
        return false;
    }

    /*
        @Filter implementors: reimplement this function to return true if your
        filter needs to match the image topic size on startup
    */
    virtual bool lockToImageSize(void)
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
            std::this_thread::sleep_for(15666us);
        }
    }

    /*
        called by Insitu
    */
    void start(QGraphicsItem* item)
    {
        filterWatchdog.setGraphicsItem(item);
        std::future<void> exitCond = exitObj.get_future();
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

    void save(Json::Value& json)
    {
        json = settings;
        json["setToImageSize"] = property(insitu::setToImageSize);
        json["keepAspectRatio"] = property(insitu::keepAspectRatio);
        json["lockFilterProperties"] = property(insitu::lockFilterProperties);
    }

    void restore(Json::Value& json)
    {
        settings = json;
        setProperty(insitu::setToImageSize,
                    json.get("setToImageSize", false).asBool());
        setProperty(insitu::keepAspectRatio,
                    json.get("keepAspectRatio", false).asBool());
        setProperty(insitu::lockFilterProperties,
                    json.get("lockFilterProperties", false).asBool());
    }

    void setProperty(insitu::FilterProps prop, bool val = true)
    {
        if (val)
        {
            props |= prop;
        }
        else
        {
            props &= ~prop;
        }
    }

    bool property(insitu::FilterProps prop) const
    {
        return (props & prop);
    }

    const std::string& name(void) const
    {
        return getName();
    }

    const std::string& getType(void) const
    {
        return type;
    }

    void setType(const std::string& type)
    {
        this->type = type;
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

    FilterWatchdog* getFilterWatchDog(void)
    {
        return &filterWatchdog;
    }

    const std::string& imageTopic(void) const
    {
        return filterWatchdog.imageTopic();
    }

    QGraphicsItem* getGraphicsItem(void) const
    {
        return filterWatchdog.getGraphicsItem();
    }

    void onInit(void)
    {
        nodelet::V_string argv = getMyArgv();
        filterWatchdog.setImageTopic(argv[0]);
        size = QSize(0, 0);

        filterInit();
    }

protected:
    /*
        @Filter implementors:  must override this function with any
       initialization required; the creating of data structures, global
       variables, etc. All initialization of the ROS infrastructure must be put
       into this function.
    */
    virtual void filterInit(void) = 0;

    /*
        @Filter implementors: reimplement this function to include any cleanup
        operations before filter shutdown
    */
    virtual void onDelete(void){};

};    // class Filter

}    // namespace insitu

#endif
