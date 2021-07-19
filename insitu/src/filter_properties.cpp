#include "filter_properties.hpp"
#include <cfloat>
#include <climits>

namespace insitu
{
/*
    Constructor/Destructor
*/
FilterProperties::FilterProperties(FilterGraphicsView* filterView, QWidget* parent)
    : QWidget(parent)
{
    /* data */
    this->filterView = filterView;

    /* ui elements */
    header = new QLabel(tr("Filter Properties"), this);
    
    sizeHeader = new QLabel(tr("Size"), this);
    widthLabel = new QLabel(tr("Width"), this);
    widthSpinBox = new QSpinBox(this);
    widthSpinBox->setMaximum(INT_MAX);
    heightLabel = new QLabel(tr("Height"), this);
    heightSpinBox = new QSpinBox(this);
    heightSpinBox->setMaximum(INT_MAX);
    aspectRatioCheckBox = new QCheckBox(tr("Keep Aspect Ratio"), this);
    setImageSizeCheckBox = new QCheckBox(tr("Set to Image Size"), this);
    
    posHeader = new QLabel(tr("Position"), this);
    xLabel = new QLabel(tr("X"), this);
    xSpinBox = new QDoubleSpinBox(this);
    xSpinBox->setMinimum(DBL_MIN);
    xSpinBox->setMaximum(DBL_MAX);
    yLabel = new QLabel(tr("Y"), this);
    ySpinBox = new QDoubleSpinBox(this);
    ySpinBox->setMaximum(DBL_MAX);
    ySpinBox->setMinimum(DBL_MIN);

    lockFilterCheckBox = new QCheckBox(tr("Lock Filter Properties"), this);

    /* layout */
    layout = new QGridLayout(this);
    layout->addWidget(header, 0, 0, 1, 2, Qt::AlignCenter);

    layout->addWidget(sizeHeader, 1, 0);
    layout->addWidget(widthLabel, 2, 0);
    layout->addWidget(widthSpinBox, 2, 1);
    layout->addWidget(heightLabel, 3, 0);
    layout->addWidget(heightSpinBox, 3, 1);
    layout->addWidget(aspectRatioCheckBox, 4, 0, 1, 2);
    layout->addWidget(setImageSizeCheckBox, 5, 0, 1, 2);

    layout->addWidget(posHeader, 6, 0, 1, 2);
    layout->addWidget(xLabel, 7, 0);
    layout->addWidget(xSpinBox, 7, 1);
    layout->addWidget(yLabel, 8, 0);
    layout->addWidget(ySpinBox, 8, 1);

    layout->addWidget(lockFilterCheckBox, 9, 0, 1, 2);
    layout->setRowStretch(9, 1);

    setLayout(layout);

    /* callbacks */
    connect(filterView->scene(), SIGNAL(selectionChanged()), 
            this, SLOT(onSelectionChanged()));

    connect(widthSpinBox, SIGNAL(valueChanged(int)),
            this, SLOT(onWidthChanged(int)));

    connect(heightSpinBox, SIGNAL(valueChanged(int)),
            this, SLOT(onHeightChanged(int)));

    connect(xSpinBox, SIGNAL(valueChanged(qreal)),
            this, SLOT(onXChanged(qreal)));

    connect(ySpinBox, SIGNAL(valueChanged(qreal)),
            this, SLOT(onYChanged(qreal)));

    connect(setImageSizeCheckBox, SIGNAL(stateChanged(int)),
            this, SLOT(onSetImageSizeChanged(int)));

    setDisabled();
}

/*
    Private Slots
*/
void FilterProperties::onSelectionChanged(void)
{
    QList<QGraphicsItem *> sel = filterView->scene()->selectedItems();
    setDisabled(sel.isEmpty());
    if (!sel.isEmpty()) {
        activeFilterItem = (FilterGraphicsItem *) sel.first();
        boost::shared_ptr<insitu::Filter> filter = activeFilterItem->getFilter();
        widthSpinBox->setValue(filter->width());
        heightSpinBox->setValue(filter->height());
        xSpinBox->setValue(activeFilterItem->x());
        ySpinBox->setValue(activeFilterItem->y());

        connect(activeFilterItem, SIGNAL(moved(QPointF)),
                this, SLOT(onFilterMoved(QPointF)));
    }
}

void FilterProperties::onWidthChanged(int w)
{
    if (aspectRatioCheckBox->isChecked()) {
        QSize s(lastWidth, lastHeight);
        s.scale(w, 0, Qt::KeepAspectRatioByExpanding);
        activeFilterItem->getFilter()->setSize(s);
        lastHeight = s.height();
        heightSpinBox->setValue(s.height());
    } else {
        activeFilterItem->getFilter()->setWidth(w);
    }
    lastWidth = w;
}

void FilterProperties::onHeightChanged(int h)
{
    if (aspectRatioCheckBox->isChecked()) {
        QSize s(lastWidth, lastHeight);
        s.scale(0, h, Qt::KeepAspectRatioByExpanding);
        activeFilterItem->getFilter()->setSize(s);
        lastWidth = s.width();
        widthSpinBox->setValue(s.width());
    } else {
        activeFilterItem->getFilter()->setHeight(h);
    }
    lastHeight = h;
}

void FilterProperties::onXChanged(qreal x)
{
    activeFilterItem->setX(x);
}

void FilterProperties::onYChanged(qreal y)
{
    activeFilterItem->setY(y);
}

void FilterProperties::onFilterMoved(QPointF pos) {
    xSpinBox->setValue(pos.x());
    ySpinBox->setValue(pos.y());
}

void FilterProperties::onSetImageSizeChanged(int state)
{
    bool setImageSize = setImageSizeCheckBox->isChecked();
    if (setImageSize) {
        aspectRatioCheckBox->setChecked(false);
        boost::shared_ptr<insitu::Filter> filter = activeFilterItem->getFilter();
        filter->setSize(filterView->getRootSize());
        widthSpinBox->setValue(filter->width());
        heightSpinBox->setValue(filter->height());
        activeFilterItem->setPos(QPointF(0,0));
    }
    widthSpinBox->setDisabled(setImageSize);
    heightSpinBox->setDisabled(setImageSize);
    aspectRatioCheckBox->setDisabled(setImageSize);
}

void FilterProperties::onLockFilterChanged(int state)
{
    
}

/*
    Public Functions
*/
void FilterProperties::setDisabled(bool disable) {
    widthSpinBox->setDisabled(disable);
    heightSpinBox->setDisabled(disable);
    aspectRatioCheckBox->setDisabled(disable);
    setImageSizeCheckBox->setDisabled(disable);
    xSpinBox->setDisabled(disable);
    ySpinBox->setDisabled(disable);
    lockFilterCheckBox->setDisabled(disable);
}

}    // namespace insitu

