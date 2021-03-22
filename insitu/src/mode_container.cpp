#include "mode_container.hpp"

namespace insitu
{
/*
    Constructor/Destructor
*/
ModeContainer::ModeContainer(QString name, QWidget* parent) : QWidget(parent)
{
    // ui elements
    addViewButton = new QPushButton(tr("Add View"));
    tileButton = new QPushButton(tr("Tile Windows"));
    cascadeButton = new QPushButton(tr("Cascade Windows"));

    container = new QMdiArea();
    container->setActivationOrder(QMdiArea::ActivationHistoryOrder);

    // callbacks
    QObject::connect(tileButton, SIGNAL(clicked()), SLOT(tile()));
    QObject::connect(cascadeButton, SIGNAL(clicked()), SLOT(cascade()));

    // layout
    layout = new QGridLayout();
    layout->addWidget(addViewButton, 0, 0);
    layout->addWidget(tileButton, 0, 2);
    layout->addWidget(cascadeButton, 0, 3);
    layout->addWidget(container, 1, 0, 1, 4);
    layout->setRowStretch(1, 1);

    layout->setColumnStretch(0, 1);

    nh = new ros::NodeHandle(name.toStdString());

    setLayout(layout);
}

void ModeContainer::tile(void)
{
    container->tileSubWindows();
}

void ModeContainer::cascade(void)
{
    container->cascadeSubWindows();
}

ModeContainer::~ModeContainer(void)
{
    delete nh;
}

void ModeContainer::addView(FilteredView* view)
{
    container->addSubWindow(view);
    view->show();
    container->tileSubWindows();
}

const ros::NodeHandle& ModeContainer::getNodeHandle(void)
{
    return *nh;
}

}    // namespace insitu
