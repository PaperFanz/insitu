#include "mode_container.hpp"
#include "add_view_dialog.hpp"

namespace insitu
{
/*
    Constructor/Destructor
*/
ModeContainer::ModeContainer(QString _name, QWidget* parent) : QWidget(parent)
{
    // ui elements
    addViewButton = new QPushButton(tr("Add View"), this);
    tileButton = new QPushButton(tr("Tile Windows"), this);
    cascadeButton = new QPushButton(tr("Cascade Windows"), this);

    container = new QMdiArea(this);
    container->setActivationOrder(QMdiArea::ActivationHistoryOrder);

    if (_name == "Tutorial") {
        QTextBrowser* tutTextBrowser = new QTextBrowser(this);
        tutTextBrowser->setOpenExternalLinks(true);
        tutTextBrowser->setSource(QUrl("qrc:/docs/tutorial.html"));
        tutTextBrowser->setStyleSheet("QTextBrowser { padding: 10 300; background:white}");
        container->addSubWindow(tutTextBrowser);
        tutTextBrowser->show();
        container->tileSubWindows();
    }

    // callbacks
    QObject::connect(addViewButton, SIGNAL(clicked()), SLOT(openViewDialog()));
    QObject::connect(tileButton, SIGNAL(clicked()), SLOT(tile()));
    QObject::connect(cascadeButton, SIGNAL(clicked()), SLOT(cascade()));

    // layout
    layout = new QGridLayout(this);
    layout->addWidget(addViewButton, 0, 0);
    layout->addWidget(tileButton, 0, 2);
    layout->addWidget(cascadeButton, 0, 3);
    layout->addWidget(container, 1, 0, 1, 4);
    layout->setRowStretch(1, 1);

    layout->setColumnStretch(0, 1);

    /* data */
    name = _name.toStdString();
    addNamedWidget("mode_" + name, this);
    nh = new ros::NodeHandle(name);

    setLayout(layout);
}

ModeContainer::ModeContainer(const Json::Value& json, QWidget* parent)
    : ModeContainer(QString::fromStdString(json.get("name", "").asString()),
                    parent)
{
    restore(json);
}

ModeContainer::~ModeContainer(void)
{
    delete nh;
}

/* public slots */
void ModeContainer::tile(void)
{
    container->tileSubWindows();
}

void ModeContainer::cascade(void)
{
    container->cascadeSubWindows();
}

void ModeContainer::openViewDialog(void)
{
    AddViewDialog* avd = (AddViewDialog*)getNamedWidget("add_view_dialog");
    avd->open();
}

/* public functions */
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

void ModeContainer::save(Json::Value& json) const
{
    json["name"] = name;
    QList<QMdiSubWindow*> viewList =
        container->subWindowList(QMdiArea::StackingOrder);
    for (int i = 0; i < viewList.size(); ++i)
    {
        FilteredView* view = static_cast<FilteredView*>(viewList[i]->widget());
        Json::Value viewJson;
        view->save(viewJson);
        json["views"].append(viewJson);
    }
}

void ModeContainer::restore(const Json::Value& json)
{
    name = json.get("name", "").asString();
    if (json.isMember("views"))
    {
        for (int i = 0; i < json["views"].size(); ++i)
        {
            // qDebug(("Adding " + json["views"].get(i,
            // "").asString()).c_str());
            addView(new FilteredView(getNodeHandle(), json["views"][i], this));
        }
    }
}

}    // namespace insitu
