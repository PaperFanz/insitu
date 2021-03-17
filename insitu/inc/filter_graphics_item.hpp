#ifndef insitu_FILTER_GRAPHICS_ITEM_HPP
#define insitu_FILTER_GRAPHICS_ITEM_HPP

// QT includes
#include <QtWidgets>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace insitu {

class FilterGraphicsItem : public QObject, public QGraphicsItem
{
Q_OBJECT
public:
    enum {Type = UserType + 42};

    FilterGraphicsItem(QGraphicsItem * parent = nullptr);

    int type() const override {return Type;}

    QRectF boundingRect(void) const override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget = nullptr) override;

public slots:

    void updateFilter(const cv::Mat& filter);

protected:

private:

    QImage img;

    mutable QMutex img_mutex;

};

} // namespace insitu

#endif // insitu_FILTER_GRAPHICS_ITEM_HPP
