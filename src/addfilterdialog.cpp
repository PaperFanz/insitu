#include "addfilterdialog.hpp"

namespace insitu {

AddFilterDialog::AddFilterDialog(QWidget * parent) : QDialog(parent)
{
    filterLoader = new pluginlib::ClassLoader<insitu_iface::Filter>("insitu_plugins", "insitu_iface::Filter");

    addButton = new QPushButton(tr("Add"));
    addButton->setDefault(true);
    cancelButton = new QPushButton(tr("Cancel"));
    filterList = new QListWidget();

    // callbacks
    QObject::connect(addButton, SIGNAL(clicked()), SLOT(AddFilter()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));

    layout = new QGridLayout();
    layout->addWidget(filterList, 0, 0, 1, 2);
    layout->addWidget(addButton, 1, 0);
    layout->addWidget(cancelButton, 1, 1);

    setLayout(layout);

    setWindowTitle(tr("Add Filter"));
}

void AddFilterDialog::AddFilter()
{
    accept();
}

void AddFilterDialog::open()
{
    refreshFilters();
    QDialog::open();
}

void AddFilterDialog::refreshFilters(void)
{
    filterList->clear();

    std::vector<std::string> classes = filterLoader->getDeclaredClasses();
    for (auto it = classes.begin(); it != classes.end(); ++it) {
        filterList->addItem(tr(it->c_str()));
        qDebug("%s", it->c_str());
    }
}

boost::shared_ptr<insitu_iface::Filter> AddFilterDialog::getInstance(QString filter)
{
    auto instance = filterLoader->createInstance(filter.toStdString());

    nodelet::M_string rmap;
    nodelet::V_string argv;

    return instance;
}

} // END NAMESPACE INSITU
