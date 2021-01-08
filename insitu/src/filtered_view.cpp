#include "filtered_view.hpp"
#include "addfilterdialog.hpp"

namespace insitu {

/*
    Constructor / Destructor
*/
FilteredView::FilteredView(QString _topic, QWidget * parent) : QWidget(parent)
{
    // Topic name selector
    topicBox = new QComboBox();
    topicBox->addItems(getTopicList());
    topicBox->setCurrentIndex(topicBox->findText(_topic));

    // buttons
    refreshTopicButton = new QPushButton(tr("Refresh"));
    addFilterButton = new QPushButton(tr("Add Filter"));

    // frame to preserve image aspect ratio
    imgFrame = new QFrame();
    imgFrame->setFrameStyle(QFrame::Plain | QFrame::Box);
    imgFrame->setBackgroundRole(QPalette::Base);

    // label to hold image
    imgLabel = new QLabel(imgFrame);
    imgLabel->setBackgroundRole(QPalette::Base);
    imgLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imgLabel->setScaledContents(true);

    // display statistics in lower status bar
    fpsLabel = new QLabel();
    fpsLabel->setText(tr("FPS: "));

    // panel for filters
    filterList = new QListWidget();

    // layout
    layout = new QGridLayout();
    layout->addWidget(topicBox, 0, 0);
    layout->addWidget(refreshTopicButton, 0, 1);
    layout->addWidget(addFilterButton, 0, 2);
    layout->addWidget(imgFrame, 1, 0, 1, 2);
    layout->addWidget(filterList, 1, 2);
    layout->addWidget(fpsLabel, 2, 0);

    layout->setColumnStretch(0, 1);

    lastFrameTime = ros::Time::now();
    frames = 0;

    setLayout(layout);

    QObject::connect(topicBox, SIGNAL(currentIndexChanged(const QString&)),
                     SLOT(onTopicChange(const QString&)));
    QObject::connect(refreshTopicButton, SIGNAL(clicked()),
                     SLOT(refreshTopics()));
    QObject::connect(addFilterButton, SIGNAL(clicked()),
                     SLOT(openFilterDialog()));

    onTopicChange(topicBox->currentText());
}

FilteredView::~FilteredView(void)
{
    sub.shutdown();
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
    AddFilterDialog * afd = (AddFilterDialog *) getNamedWidget("addfilterdialog");
    afd->setActiveView(this);
    afd->open();
}

void FilteredView::onTopicChange(QString topic_transport)
{
    // qDebug("topic changed");

    // imgMat.release();

    QList<QString> l = topic_transport.split(" ");
    std::string topic = l.first().toStdString();
    std::string transport = l.length() == 2 ? l.last().toStdString() : "raw";

    if (!topic.empty()) {
        if(sub.getNumPublishers()) sub.shutdown();
        
        image_transport::ImageTransport it(insituNodeHandle());
        image_transport::TransportHints hints(transport);

        sub = it.subscribe(topic, 1, &FilteredView::callbackImg, this, hints);
    } else {
        // TODO error message
    }

    // qDebug("Subscribed to topic %s / %s", sub.getTopic().c_str(), sub.getTransport().c_str());
}

/*
    Public Functions
*/
void FilteredView::addFilter(boost::shared_ptr<insitu::Filter> filter)
{
    std::string name = filter->name();

    if (filters.find(name) == filters.end()) {
        filterOrder.push_back(name);
        filters[name] = filter;
    } else {
        // TODO err filter exists
    }
}

/*
    Private Functions
*/
const QSize hacky_shit(2,2);

void FilteredView::callbackImg(const sensor_msgs::Image::ConstPtr& msg)
{
    // qDebug("cb in");
    // track frames per second
    ros::Time now = ros::Time::now();
    ++frames;
    if (now - lastFrameTime > ros::Duration(1)) {
        fpsLabel->setText(QString("FPS: %1").arg(frames));
        frames = 0;
        lastFrameTime = now;
    }

    // convert sensor_msgs::Image to cv matrix
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        qWarning("Failed to convert image: %s", e.what());
        return;
    }
    imgMat = cv_ptr->image;

    // apply filters
    for (auto it = filterOrder.begin(); it != filterOrder.end(); ++it) {
        imgMat = filters[*it]->apply(imgMat);
    }

    // convert cv matrix to qpixmap
    QImage image(imgMat.data, imgMat.cols, imgMat.rows, imgMat.step[0], QImage::Format_RGB888);
    QPixmap img = QPixmap::fromImage(image);

    // maximize while preserving aspect ratio
    QSize s = img.size();
    s.scale(imgFrame->size() - hacky_shit, Qt::KeepAspectRatio);
    imgLabel->resize(s);
    imgLabel->setPixmap(img);
    // qDebug("cb out");
}

}
