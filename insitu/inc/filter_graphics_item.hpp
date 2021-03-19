#ifndef insitu_FILTER_GRAPHICS_ITEM_HPP
#define insitu_FILTER_GRAPHICS_ITEM_HPP

// QT includes
#include <QtWidgets>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace insitu
{
class FilterGraphicsItem : public QObject, public QGraphicsItem
{
    /*
        Why the hell does a QGraphicsItem also subclass QObject? It's a sad
       story.

        For some unknown reason simply calling update() after updating the image
       in updateFilter() results in the QGraphicsItem freezing after a while.
       The only solution I found that worked came from rqt_image_view commit
       e7c4c25, using signals and slots to separately queue the call to
       update(). To anyone who sees this, a cleaner solution that does not
       require subclassing QObject could reduce memory usage by up to 30% and is
       always welcome.
    */
    Q_OBJECT
public:
    enum
    {
        Type = UserType + 42
    };

    FilterGraphicsItem(QGraphicsItem* parent = nullptr);

    int type(void) const override
    {
        return Type;
    }

    QRectF boundingRect(void) const override;

    QSize getImgSize(void) const;

    void updateFilter(const cv::Mat& filter);

    void setResizable(bool resizeable);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;

    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget = nullptr) override;

public slots:

    void queuedUpdate(void);

signals:

    void delayedUpdate(void);

    void imgSizeChanged(QSize size);

private:
    QPointF itemCenter(void) const;

    QImage img;

    QSize imgSize;

    mutable QMutex img_mutex;

    bool isResizable;

    bool isResizing;

    double initDist;
    double initScale;
};

}    // namespace insitu

#endif    // insitu_FILTER_GRAPHICS_ITEM_HPP
