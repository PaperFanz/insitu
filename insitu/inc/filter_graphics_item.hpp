#ifndef insitu_FILTER_GRAPHICS_ITEM_HPP
#define insitu_FILTER_GRAPHICS_ITEM_HPP

// QT includes
#include <qlistwidget.h>
#include <QtWidgets>

// OpenCV includes
#include <boost/smart_ptr/shared_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

/* insitu includes */
#include "lib/insitu/filter.hpp"

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

    FilterGraphicsItem(boost::shared_ptr<insitu::Filter> filter = nullptr,
                       QListWidgetItem* listItem = nullptr,
                       QGraphicsItem* parent = nullptr);

    int type(void) const override
    {
        return Type;
    }

    QRectF boundingRect(void) const override;

    QSize getImgSize(void) const;

    void updateFilter(const cv::Mat& filter);

    boost::shared_ptr<insitu::Filter> getFilter(void) const;

    QListWidgetItem* listItem(void) const;

protected:
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget = nullptr) override;

public slots:
    void queuedUpdate(void);

    void onVisibilityChanged(bool visible);

signals:
    void delayedUpdate(void);

    void imgSizeChanged(QSize size);

    void moved(QPointF pos);

private:
    boost::shared_ptr<insitu::Filter> filter_;

    QListWidgetItem* listItem_;

    QPointF itemCenter(void) const;

    QImage img;

    QSize imgSize;

    mutable QMutex img_mutex;
};

}    // namespace insitu

#endif    // insitu_FILTER_GRAPHICS_ITEM_HPP
