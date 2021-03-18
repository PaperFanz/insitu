#include "filter_graphics_view.hpp"

namespace insitu
{

/*
    Constructor
*/
FilterGraphicsView::FilterGraphicsView(QGraphicsScene * scene, 
    QWidget * parent) : QGraphicsView(scene, parent)
{
    root = nullptr;
    imgBuf = QImage(1,1, QImage::Format_ARGB32);
}

/*
    Public Functions
*/
void FilterGraphicsView::setRootItem(FilterGraphicsItem * item)
{
    if (item == nullptr) return;
    root = item;
    root->setFlag(QGraphicsItem::ItemIsMovable, false);
    root->setFlag(QGraphicsItem::ItemIsSelectable, false);
    root->setFlag(QGraphicsItem::ItemSendsGeometryChanges, false);
    root->setResizable(false);

    connect(root, SIGNAL(imgSizeChanged(QSize)),
            this, SLOT(rootImgSizeChanged(QSize)));
}

void FilterGraphicsView::fitToRoot(void)
{
    if (root != nullptr) {
        fitInView(root, Qt::KeepAspectRatio);
        setSceneRect(root->boundingRect());
    }
}

void FilterGraphicsView::setReplublishing(bool repub)
{
    republishing = repub;
    if (republishing) {
        setViewportUpdateMode(QGraphicsView::NoViewportUpdate);
        setInteractive(false);
    } else {
        setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
        setInteractive(true);
    }
}

const QImage & FilterGraphicsView::getImage(void)
{
    QPainter imgPainter(&imgBuf);
    if (republishing) scene()->render(&imgPainter);
    return imgBuf;
}

/*
    Public Slots
*/
void FilterGraphicsView::rootImgSizeChanged(QSize size)
{
    imgBuf = QImage(root->getImgSize(), QImage::Format_RGBA8888);
}

/*
    Reimplemented Protected Functions
*/
void FilterGraphicsView::resizeEvent(QResizeEvent * event)
{
    QGraphicsView::resizeEvent(event);
    fitToRoot();
}

} // namespace insitu
