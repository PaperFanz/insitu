#include "filtered_view.hpp"

namespace insitu {

FilteredView::FilteredView(QString _topic, QWidget * parent) : QWidget(parent)
{
    // Topic name selector
    topicBox = new QComboBox();
    topicBox->addItems(getTopicList());
    int idx = topicBox->findText(_topic);
    QObject::connect(topicBox, SIGNAL(currentIndexChanged(const QString&)),
                     SLOT(onTopicChange(const QString&)));
    topicBox->setCurrentIndex(idx);
    onTopicChange(topicBox->currentText());

    // button to refresh topic list
    refreshTopicButton = new QPushButton(tr("Refresh"));
    QObject::connect(refreshTopicButton, SIGNAL(clicked()),
                     SLOT(refreshTopics()));

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
    imgPan->setWidgetResizable(true);
    // imgPan->setVisible(false);

    vBox = new QVBoxLayout();
    vBox->addLayout(menuBar);
    vBox->addWidget(imgPan);

    setLayout(vBox);
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
        image_transport::ImageTransport it(nh);
        image_transport::TransportHints hints(transport);

        if (init) init = false;
        else sub.shutdown();

        sub = it.subscribe(topic, 1, &FilteredView::callbackImg, this, hints);
    } else {
        // TODO error message
    }

    qDebug("Subscribed to topic %s / %s", sub.getTopic().c_str(), sub.getTransport().c_str());
}

void FilteredView::callbackImg(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    imgMat = cv_ptr->image;

    QImage image(imgMat.data, imgMat.cols, imgMat.rows, imgMat.step[0], QImage::Format_RGB888);
    QPixmap img = QPixmap();
    if (img.convertFromImage(image)) {
        imgLabel->resize(img.size());
        imgLabel->setPixmap(img);
    } else {
        qWarning("Failed to load pixmap");
    }
}

}
