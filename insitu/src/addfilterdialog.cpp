#include "addfilterdialog.hpp"

namespace insitu {

AddFilterDialog::AddFilterDialog(QWidget * parent) : QDialog(parent)
{
    filterLoader = new FilterFactory("insitu");
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

    std::vector<std::string> classes = filterLoader->getFilterList();
    for (auto it = classes.begin(); it != classes.end(); ++it) {
        filterList->addItem(tr(it->c_str()));
        qDebug("%s", it->c_str());
    }
}

} // END NAMESPACE INSITU
