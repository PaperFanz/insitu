#include "filtered_view.hpp"
#include "add_filter_dialog.hpp"
#include "filter_card.hpp"
#include "filter_graphics_item.hpp"

namespace insitu
{
/*
    Constructor / Destructor
*/
FilteredView::FilteredView(const ros::NodeHandle& parent_, QString _name,
                           QString _topic, QWidget* parent)
    : QWidget(parent)
{
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
    filterPaneSplitter->setStyleSheet("QSplitter::handle { color: #000 }"); // TODO get a nice image for the splitter handle
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

    errMsg = new QErrorMessage();

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

    name = _name.toStdString();
    nh = new ros::NodeHandle(parent_, name);
    nh->setCallbackQueue(&viewQueue);
    spinner = new ros::AsyncSpinner(1, &viewQueue);
    spinner->start();

    onTopicChange(topicBox->currentText());
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
        // TODO error message
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
    connect(filter->getFilterWatchDog(),
            SIGNAL(filterUpdated(QGraphicsItem*, const cv::Mat&)), this,
            SLOT(updateFilter(QGraphicsItem*, const cv::Mat&)));
}

const std::string& FilteredView::getViewName(void) const
{
    return name;
}

const ros::NodeHandle& FilteredView::getNodeHandle(void) const
{
    return *nh;
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

    AddFilterDialog* afd =
        (AddFilterDialog*)getNamedWidget("add_filter_dialog");
    afd->unloadFilter(filterName);
}

}    // namespace insitu
