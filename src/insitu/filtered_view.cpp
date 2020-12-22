#include "filtered_view.hpp"

namespace insitu {

FilteredView::FilteredView(QString _topic, QWidget * parent) : QWidget(parent)
{
    topic = _topic;

    // Topic name selector
    topicBox = new QComboBox();
    topicBox->addItems(getTopicList());

    // button to refresh topic list
    refreshTopicButton = new QPushButton(tr("Refresh"));
    QObject::connect(refreshTopicButton, SIGNAL(clicked()), SLOT(refreshTopics()));

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

    setWindowTitle(_topic);
}

void FilteredView::refreshTopics(void)
{
    topicBox->clear();
    topicBox->addItems(getTopicList());
}

void FilteredView::callbackImg(const sensor_msgs::Image::ConstPtr& msg)
{
    
}

}
