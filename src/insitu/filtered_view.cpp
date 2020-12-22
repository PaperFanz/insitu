#include "filtered_view.hpp"

namespace insitu {

FilteredView::FilteredView(QString _topic, QWidget * parent) : QWidget(parent)
{
    // Topic name selector
    topicBox = new QComboBox();
    topicBox->addItems(getTopicList());
    int idx = topicBox->findText(_topic);
    topicBox->setCurrentIndex(idx);
    onTopicChange(topicBox->currentText());

    // button to refresh topic list
    refreshTopicButton = new QPushButton(tr("Refresh"));

    // layout
    menuBar = new QHBoxLayout();
    menuBar->addWidget(topicBox);
    menuBar->addWidget(refreshTopicButton);

    // label to hold image
    imgLabel = new QLabel();
    imgLabel->setBackgroundRole(QPalette::Base);
    imgLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imgLabel->setScaledContents(true);

    // scroll area for panning zoomed image
    imgPan = new QScrollArea();
    imgPan->setBackgroundRole(QPalette::Dark);
    imgPan->setWidget(imgLabel);
    // imgPan->setVisible(false);

    vBox = new QVBoxLayout();
    vBox->addLayout(menuBar);
    vBox->addWidget(imgPan);

    setLayout(vBox);

    QObject::connect(topicBox, SIGNAL(currentIndexChanged(const QString&)),
                     SLOT(onTopicChange(const QString&)));
    QObject::connect(refreshTopicButton, SIGNAL(clicked()),
                     SLOT(refreshTopics()));
}

FilteredView::~FilteredView(void)
{
    sub.shutdown();
}

void FilteredView::refreshTopics(void)
{
    QString save_topic = topicBox->currentText();
    topicBox->clear();
    topicBox->addItems(getTopicList());
    int idx = topicBox->findText(save_topic);
    idx = idx == -1 ? 0 : idx;
    topicBox->setCurrentIndex(idx);
}

void FilteredView::onTopicChange(QString topic_transport)
{
    static bool init = true;

    // imgMat.release();

    QList<QString> l = topic_transport.split(" ");
    std::string topic = l.first().toStdString();
    std::string transport = l.length() == 2 ? l.last().toStdString() : "raw";

    if (!topic.empty()) {
        if (init) init = false;
        else sub.shutdown();
        
        image_transport::ImageTransport it(nh);
        image_transport::TransportHints hints(transport);

        sub = it.subscribe(topic, 1, &FilteredView::callbackImg, this, hints);
    } else {
        // TODO error message
    }

    qDebug("Subscribed to topic %s / %s", sub.getTopic().c_str(), sub.getTransport().c_str());
}

void FilteredView::callbackImg(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        qWarning("Failed to convert image: %s", e.what());
        return;
    }

    imgMat = cv_ptr->image;
    QImage image(imgMat.data, imgMat.cols, imgMat.rows, imgMat.step[0], QImage::Format_RGB888);
    QPixmap img = QPixmap::fromImage(image);
    imgLabel->resize(img.size());
    imgLabel->setPixmap(img);
    qDebug("Callback on view %s", this->windowTitle().toStdString().c_str());
}

}
