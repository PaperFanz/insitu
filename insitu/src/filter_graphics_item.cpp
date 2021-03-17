#include "filter_graphics_item.hpp"

namespace insitu
{

/*
    Constructor
*/
FilterGraphicsItem::FilterGraphicsItem(QGraphicsItem * parent) 
    : QGraphicsItem(parent)
{
    img = QImage(0,0, QImage::Format_RGBA8888);
    setFlag(QGraphicsItem::ItemIsMovable, true);
    setFlag(QGraphicsItem::ItemIsSelectable, true);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
}

/*
    Slots
*/
void FilterGraphicsItem::updateFilter(const cv::Mat& filter)
{
    // convert cv matrix to qpixmap
    img_mutex.lock();
    img = QImage(filter.data, filter.cols, filter.rows, filter.step[0], 
                 QImage::Format_RGBA8888);
    img_mutex.unlock();
    update();
}

/*
    Protected
*/

QRectF FilterGraphicsItem::boundingRect(void) const
{
    return img.rect();
}

void FilterGraphicsItem::paint(QPainter *painter, 
    const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    // TODO freezing
    img_mutex.lock();
    painter->drawImage(0, 0, img);
    img_mutex.unlock();
}

} // namespace insitu
