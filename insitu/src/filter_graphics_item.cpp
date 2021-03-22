#include "filter_graphics_item.hpp"

namespace insitu
{
/*
    Constructor
*/
FilterGraphicsItem::FilterGraphicsItem(QGraphicsItem* parent)
    : QGraphicsItem(parent)
{
    isResizable = true;
    isResizing = false;
    img = QImage(1, 1, QImage::Format_RGBA8888);

    setFlag(QGraphicsItem::ItemIsMovable, true);
    setFlag(QGraphicsItem::ItemIsSelectable, true);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);

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

void FilterGraphicsItem::setResizable(bool resizable)
{
    isResizable = resizable;
}

/*
    Protected
*/

QRectF FilterGraphicsItem::boundingRect(void) const
{
    return img.rect();
}

void FilterGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        if (event->modifiers() == Qt::ShiftModifier)
        {
            // add the item to the selection
            setSelected(true);
        }
        else if (event->modifiers() == Qt::AltModifier)
        {
            // resize the item
            QPointF center = itemCenter();
            QPointF pos = event->scenePos();

            double distx = abs(center.x() - pos.x());
            double disty = abs(center.y() - pos.y());
            if (distx > 0.4 * boundingRect().width() ||
                disty > 0.4 * boundingRect().height())
            {
                initDist = sqrt(pow(distx, 2) + pow(disty, 2));
                initScale = scale();
                isResizing = true;
            }
            else
            {
                isResizing = false;
            }
        }
        else
        {
            QGraphicsItem::mousePressEvent(event);
            event->accept();
        }
    }
    else if (event->button() == Qt::RightButton)
    {
        event->ignore();
    }
}

void FilterGraphicsItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->modifiers() == Qt::AltModifier && isResizing && isResizable)
    {
        QPointF center = itemCenter();
        QPointF pos = event->scenePos();
        double dist = sqrt(pow((center.x() - pos.x()), 2) +
                           pow((center.y() - pos.y()), 2));
        setTransformOriginPoint(center);
        setScale(initScale * dist / initDist);
    }
    else if (event->modifiers() != Qt::AltModifier)
    {
        QGraphicsItem::mouseMoveEvent(event);
    }
}

void FilterGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->modifiers() == Qt::AltModifier && isResizing)
    {
        isResizing = false;
    }
    else if (event->modifiers() != Qt::ShiftModifier)
    {
        QGraphicsItem::mouseReleaseEvent(event);
    }
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
