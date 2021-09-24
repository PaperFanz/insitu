#include "filter_graphics_item.hpp"
#include <qgraphicsitem.h>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace insitu
{
/*
    Constructor
*/
FilterGraphicsItem::FilterGraphicsItem(boost::shared_ptr<insitu::Filter> filter,
                                       QGraphicsItem* parent)
    : QGraphicsItem(parent)
{
    this->filter = filter;
    if (filter != nullptr)
    {
        setFlag(QGraphicsItem::ItemIsSelectable, true);
        setFlag(QGraphicsItem::ItemIsMovable, true);
        setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    }

    img = QImage(1, 1, QImage::Format_RGBA8888);

    // some jank shit, don't touch
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(queuedUpdate()),
            Qt::QueuedConnection);
}

/*
    Public
*/
QSize FilterGraphicsItem::getImgSize(void) const
{
    return imgSize;
}

boost::shared_ptr<insitu::Filter> FilterGraphicsItem::getFilter(void) const
{
    return filter;
}

/*
    Slots
*/
void FilterGraphicsItem::updateFilter(const cv::Mat& filter)
{
    // convert cv matrix to qpixmap
    img_mutex.lock();
    img = QImage(filter.data, filter.cols, filter.rows, filter.step[0],
                 QImage::Format_RGBA8888)
              .copy();
    img_mutex.unlock();
    if (imgSize != img.size())
    {
        imgSize = img.size();
        emit imgSizeChanged(imgSize);
    }
    emit delayedUpdate();
}

void FilterGraphicsItem::queuedUpdate(void)
{
    update(boundingRect());
}

/*
    Protected
*/

QRectF FilterGraphicsItem::boundingRect(void) const
{
    return img.rect();
}

void FilterGraphicsItem::paint(QPainter* painter,
                               const QStyleOptionGraphicsItem* option,
                               QWidget* widget)
{
    img_mutex.lock();
    painter->drawImage(0, 0, img);
    img_mutex.unlock();
    if (isSelected())
    {
        QPen pen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter->setPen(pen);
        painter->drawRect(img.rect());
    }
}

/*
    Private
*/
QPointF FilterGraphicsItem::itemCenter(void) const
{
    return QPointF(
        (boundingRect().topLeft().x() + boundingRect().bottomRight().x()) / 2,
        (boundingRect().topLeft().y() + boundingRect().bottomRight().y()) / 2);
}

}    // namespace insitu
