#ifndef insitu_FILTER_PROPERTIES_HPP
#define insitu_FILTER_PROPERTIES_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "filter_graphics_item.hpp"
#include "filter_graphics_view.hpp"
#include "insitu_utils.hpp"

namespace insitu
{
class FilterProperties : public QWidget
{
    Q_OBJECT
private:
    /* ui elements */
    QLabel* header;

    QLabel* sizeHeader;
    QLabel* widthLabel;
    QSpinBox* widthSpinBox;
    QLabel* heightLabel;
    QSpinBox * heightSpinBox;
    QCheckBox* aspectRatioCheckBox;
    QCheckBox* setImageSizeCheckBox;

    QLabel* posHeader;
    QLabel* xLabel;
    QDoubleSpinBox* xSpinBox;
    QLabel* yLabel;
    QDoubleSpinBox* ySpinBox;

    QCheckBox* lockFilterCheckBox;

    // layout element
    QGridLayout* layout;

    /* data */
    FilterGraphicsView * filterView;
    FilterGraphicsItem * activeFilterItem;

    QSize savedSize;

    double aspectRatio; // width/height

    int lastWidth;
    int lastHeight;

private Q_SLOTS:
    void onSelectionChanged(void);

    void onWidthChanged(int w);

    void onHeightChanged(int h);

    void onXChanged(qreal x);

    void onYChanged(qreal y);

    void onFilterMoved(QPointF pos);

    void onAspectRatioChanged(int state);

    void onSetImageSizeChanged(int state);

    void onLockFilterChanged(int state);

public:
    FilterProperties(FilterGraphicsView* filterView, QWidget* parent = nullptr);

    void setDisabled(bool disable = true);

};

}    // end namespace insitu

#endif // insitu_FILTER_PROPERTIES_HPP

