#include "ros_image_frame.hpp"

namespace insitu
{
/*
    Constructor/Destructor
*/
RosImageFrame::RosImageFrame(QWidget* parent) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()),
            Qt::QueuedConnection);
}

RosImageFrame::~RosImageFrame(void)
{
}

/*
    Public Functions
*/
void RosImageFrame::setImage(const QImage& img_)
{
    img_mutex.lock();
    img = img_.copy();
    img_mutex.unlock();
    emit delayedUpdate();
}

/*
    Protected Functions
*/
void RosImageFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    img_mutex.lock();

    if (!img.isNull())
    {
        QImage scaled = img.scaled(contentsRect().size(), Qt::KeepAspectRatio,
                                   Qt::SmoothTransformation);
        painter.drawImage(0, 0, scaled);
    }

    img_mutex.unlock();
}

}    // namespace insitu
