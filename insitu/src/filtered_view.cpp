#include "filtered_view.hpp"
#include <string>
#include "add_filter_dialog.hpp"
#include "filter_card.hpp"
#include "filter_graphics_item.hpp"
#include "insitu_utils.hpp"

namespace insitu
{
/*
    Constructor / Destructor
*/
FilteredView::FilteredView(const ros::NodeHandle& parent_, QString _name,
                           QString _topic, QWidget* parent)
    : QWidget(parent)
{
    /* decorations */
    setWindowTitle(_name);

    // Topic name selector
    topicBox = new QComboBox();
    topicBox->addItems(getTopicList());
    topicBox->setCurrentIndex(topicBox->findText(_topic));

    // buttons
    refreshTopicButton = new QPushButton(tr("Refresh"));
    addFilterButton = new QPushButton(tr("Add Filter"));
    rmFilterButton = new QPushButton(tr("Delete Filter"));

    // checkboxes
    showFilterPaneCheckBox = new QCheckBox(tr("Show Filter Pane"));
    showFilterPaneCheckBox->setChecked(true);
    republishCheckBox = new QCheckBox(tr("Republish"));

    // display statistics in lower status bar
    fpsLabel = new QLabel(tr("FPS: "), this);

    // graphics view for rendering filters
    filterScene = new QGraphicsScene(this);
    filterScene->setBackgroundBrush(QBrush(Qt::lightGray));
    filterView = new FilterGraphicsView(filterScene, this);
    rosImg = new FilterGraphicsItem();
    filterScene->addItem(rosImg);
    filterView->setRootItem(rosImg);

    // panel for filters
    filterList = new QListWidget(this);
    filterList->setDragDropMode(QAbstractItemView::DragDrop);
    filterList->setDefaultDropAction(Qt::DropAction::MoveAction);
    filterProps = new FilterProperties(filterView, this);
    filterPaneSplitter = new QSplitter(Qt::Vertical);
    filterPaneSplitter->setStyleSheet(
        "QSplitter::handle { color: #000 }");    // TODO get a nice image for
                                                 // the splitter handle
    filterPaneSplitter->addWidget(filterList);
    filterPaneSplitter->addWidget(filterProps);
    filterPaneSplitter->setStretchFactor(0, 10);

    // layout
    filterPaneLayout = new QGridLayout();
    filterPaneLayout->addWidget(addFilterButton, 0, 0);
    filterPaneLayout->addWidget(rmFilterButton, 0, 1);
    filterPaneLayout->addWidget(filterPaneSplitter, 1, 0, 1, 2);
    filterPaneWidget = new QWidget();
    filterPaneWidget->setLayout(filterPaneLayout);

    imagePane = new QHBoxLayout();
    imagePane->addWidget(filterView, 1);
    imagePane->addWidget(filterPaneWidget);

    errMsg = new QErrorMessage(this);

    layout = new QGridLayout();
    layout->addWidget(topicBox, 0, 0);
    layout->addWidget(refreshTopicButton, 0, 1);
    layout->addWidget(republishCheckBox, 0, 2);
    layout->addWidget(showFilterPaneCheckBox, 0, 3);
    layout->addLayout(imagePane, 1, 0, 1, 4);
    layout->addWidget(fpsLabel, 2, 0);
    layout->setColumnStretch(0, 1);
    layout->setRowStretch(1, 1);

    lastFrameTime = ros::Time::now();
    frames = 0;

    setLayout(layout);

    connect(topicBox, SIGNAL(currentIndexChanged(const QString&)),
            SLOT(onTopicChange(const QString&)));
    connect(refreshTopicButton, SIGNAL(clicked()), SLOT(refreshTopics()));
    connect(addFilterButton, SIGNAL(clicked()), SLOT(openFilterDialog()));
    connect(rmFilterButton, SIGNAL(clicked()), SLOT(rmFilter()));
    connect(showFilterPaneCheckBox, SIGNAL(stateChanged(int)),
            SLOT(onToggleFilterPane()));
    connect(republishCheckBox, SIGNAL(stateChanged(int)),
            SLOT(onToggleRepublish()));
    connect(filterList, SIGNAL(itemSelectionChanged()),
            SLOT(onFilterOrderChanged()));

    /* ROS initializations */
    filterFactory = new FilterFactory();
    name = _name.toStdString();
    addNamedWidget("view_" + name, this);
    nh = new ros::NodeHandle(parent_, name);
    nh->setCallbackQueue(&viewQueue);
    spinner = new ros::AsyncSpinner(1, &viewQueue);
    spinner->start();

    onTopicChange(topicBox->currentText());
}

FilteredView::FilteredView(const ros::NodeHandle& parent_,
                           const Json::Value& json, QWidget* parent)
    : FilteredView(
          parent_, QString::fromStdString(json.get("name", "").asString()),
          QString::fromStdString(json.get("topic", "").asString()), parent)
{
    restore(json);
}

FilteredView::~FilteredView(void)
{
    for (int i = filterList->count() - 1; i >= 0; --i)
    {
        QListWidgetItem* item = filterList->item(i);
        unloadFilter(item);
        delete item;
    }
    sub.shutdown();
    spinner->stop();
    delete spinner;
    delete nh;
    delete filterFactory;
}

/*
    Slots
*/
void FilteredView::refreshTopics(void)
{
    QString save_topic = topicBox->currentText();
    topicBox->clear();
    topicBox->addItems(getTopicList());
    int idx = topicBox->findText(save_topic);
    idx = idx == -1 ? 0 : idx;
    topicBox->setCurrentIndex(idx);
}

void FilteredView::openFilterDialog(void)
{
    AddFilterDialog* afd =
        (AddFilterDialog*)getNamedWidget("add_filter_dialog");
    afd->setActiveView(this);
    afd->open();
}

void FilteredView::onTopicChange(QString topic_transport)
{
    QList<QString> l = topic_transport.split(" ");
    std::string topic = l.first().toStdString();
    std::string transport = l.length() == 2 ? l.last().toStdString() : "raw";

    if (!topic.empty())
    {
        if (sub.getNumPublishers()) sub.shutdown();

        image_transport::ImageTransport it(*nh);
        image_transport::TransportHints hints(transport);

        topicChanged = true;
        sub = it.subscribe(topic, 1, &FilteredView::callbackImg, this, hints);
    }
    else
    {
        errMsg->showMessage(QString::fromStdString("Topic cannot be empty!"));
    }
}

void FilteredView::rmFilter(void)
{
    QListWidgetItem* item = filterList->currentItem();
    unloadFilter(item);
    delete item;
}

void FilteredView::onToggleFilterPane(void)
{
    filterPaneWidget->setVisible(showFilterPaneCheckBox->isChecked());
}

void FilteredView::onToggleRepublish(void)
{
    if (republishCheckBox->isChecked())
    {
        topicBox->setDisabled(true);
        filterView->setReplublishing(true);
        image_transport::ImageTransport it(*nh);
        pub = it.advertise("republish", 1);
    }
    else
    {
        pub.shutdown();
        topicBox->setDisabled(false);
        filterView->setReplublishing(false);
        filterScene->update();
    }
}

void FilteredView::onFilterOrderChanged(void)
{
    for (int i = 0; i < filterList->count(); ++i)
    {
        QListWidgetItem* item = filterList->item(i);
        FilterCard* fc = (FilterCard*)filterList->itemWidget(item);
        if (fc != nullptr)
        {
            std::string filterName = fc->getFilterName();
            boost::shared_ptr<insitu::Filter> f = filters[filterName];
            f->getGraphicsItem()->setZValue(i);
        }
    }
}

void FilteredView::updateFilter(QGraphicsItem* item, const cv::Mat& update)
{
    ((FilterGraphicsItem*)item)->updateFilter(update);
}

/*
    Public Functions
*/
std::string FilteredView::getViewTopic(void) const
{
    return sub.getTopic();
}

void FilteredView::addFilter(boost::shared_ptr<insitu::Filter> filter)
{
    std::string name = filter->name();
    filters[name] = filter;

    QListWidgetItem* item = new QListWidgetItem();
    FilterCard* fc = new FilterCard(name, filter);
    item->setSizeHint(fc->sizeHint());

    filterList->addItem(item);
    filterList->setItemWidget(item, fc);

    FilterGraphicsItem* gi = new FilterGraphicsItem(filter, rosImg);
    filter->start(gi);
    qRegisterMetaType<cv::Mat>("cv::Mat");

    /* ebedded Q_OBJECT to leverage QT slots and signals */
    FilterWatchdog* wd = filter->getFilterWatchDog();
    wd->setImageTopic(sub.getTopic());

    /* async filter updates independent of main ui thread */
    connect(wd, SIGNAL(filterUpdated(QGraphicsItem*, const cv::Mat&)), this,
            SLOT(updateFilter(QGraphicsItem*, const cv::Mat&)));

    /* forward topic changes to filters that subscribe to the same base image
     * topic */
    connect(topicBox, SIGNAL(currentIndexChanged(const QString&)), wd,
            SLOT(onTopicChanged(const QString&)));
}

const std::string& FilteredView::getViewName(void) const
{
    return name;
}

const ros::NodeHandle& FilteredView::getNodeHandle(void) const
{
    return *nh;
}

void FilteredView::save(Json::Value& json) const
{
    json["name"] = name;
    json["topic"] = topicBox->currentText().toStdString();
    json["republish"] = republishCheckBox->isChecked();
    json["showFilterPane"] = showFilterPaneCheckBox->isChecked();

    for (const auto& it : filters)
    {
        Json::Value filterjson;
        filterjson["name"] = it.first;
        boost::shared_ptr<insitu::Filter> filter = it.second;
        filterjson["type"] = filter->getType();
        FilterGraphicsItem* fgitem =
            (FilterGraphicsItem*)filter->getGraphicsItem();
        filterjson["properties"]["x"] = fgitem->x();
        filterjson["properties"]["y"] = fgitem->y();
        filterjson["properties"]["width"] = filter->width();
        filterjson["properties"]["height"] = filter->height();
        filter->save(filterjson["settings"]);
        json["filters"].append(filterjson);
    }
}

void FilteredView::restore(const Json::Value& json)
{
    republishCheckBox->setChecked(json.get("republish", false).asBool());
    showFilterPaneCheckBox->setChecked(
        json.get("showFilterPane", false).asBool());

    for (int i = 0; i < json["filters"].size(); ++i)
    {
        Json::Value filterjson = json["filters"][i];
        if (!filterjson.isMember("name"))
        {
            errMsg->showMessage(QString::fromStdString(
                "Filter must be named, skipping " + std::to_string(i) + " of " +
                std::to_string(json["filters"].size())));
            continue;
        }
        if (!filterjson.isMember("type"))
        {
            errMsg->showMessage(QString::fromStdString(
                "Filter must have type, skipping " + std::to_string(i) +
                " of " + std::to_string(json["filters"].size())));
            continue;
        }
        auto filter = filterFactory->loadFilter(
            filterjson.get("type", "").asString(),
            filterjson.get("name", "").asString(), sub.getTopic());
        addFilter(filter);
        if (filterjson.isMember("properties") &&
            !filterjson["properties"].isNull())
        {
            FilterGraphicsItem* fgitem =
                (FilterGraphicsItem*)filter->getGraphicsItem();
            fgitem->setPos(
                QPointF(filterjson["properties"].get("x", 0).asDouble(),
                        filterjson["properties"].get("y", 0).asDouble()));
            filter->setSize(
                QSize(filterjson["properties"].get("width", "100").asInt(),
                      filterjson["properties"].get("height", "100").asInt()));
        }
        if (filterjson.isMember("settings") && !filterjson["settings"].isNull())
        {
            filter->restore(filterjson["settings"]);
        }
    }
}

/*
    Private Functions
*/
void FilteredView::callbackImg(const sensor_msgs::Image::ConstPtr& msg)
{
    // track frames per second
    ros::Time now = ros::Time::now();
    ++frames;
    if (now - lastFrameTime > ros::Duration(1))
    {
        fpsLabel->setText(QString("FPS: %1").arg(frames));
        frames = 0;
        lastFrameTime = now;
    }

    // convert sensor_msgs::Image to cv matrix
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);
    }
    catch (cv_bridge::Exception& e)
    {
        qWarning("Failed to convert image: %s", e.what());
        return;
    }

    rosImg->updateFilter(cv_ptr->image);
    if (topicChanged)
    {
        filterView->fitToRoot();
        topicChanged = false;
    }

    // republish
    if (pub.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage repub;
        filteredImg = filterView->getImage().copy();
        repub.encoding = "rgba8";
        repub.image = cv::Mat(filteredImg.height(), filteredImg.width(),
                              CV_8UC4, const_cast<uchar*>(filteredImg.bits()),
                              static_cast<size_t>(filteredImg.bytesPerLine()));
        pub.publish(repub.toImageMsg());
    }
}

void FilteredView::unloadFilter(QListWidgetItem* filterItem)
{
    if (filterItem == nullptr) return;
    FilterCard* fc = (FilterCard*)filterList->itemWidget(filterItem);
    std::string filterName = fc->getFilterName();

    delete fc;

    boost::shared_ptr<insitu::Filter> f = filters[filterName];
    filterScene->removeItem(f->getGraphicsItem());
    f->stop();
    filters.erase(filterName);

    if (!filterFactory->unloadFilter(filterName))
    {
        AddFilterDialog* afd =
            (AddFilterDialog*)getNamedWidget("add_filter_dialog");
        if (!afd->unloadFilter(filterName))
        {
            throw std::runtime_error("Failed to unload " + filterName);
        }
    }
}

}    // namespace insitu
