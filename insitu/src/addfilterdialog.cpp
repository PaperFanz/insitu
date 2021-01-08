#include "addfilterdialog.hpp"

namespace insitu {

/*
    Constructor
*/
AddFilterDialog::AddFilterDialog(QWidget * parent) : QDialog(parent)
{
    filterLoader = new FilterFactory("insitu");
    activeView = nullptr;

    addButton = new QPushButton(tr("Add"));
    addButton->setDefault(true);

    cancelButton = new QPushButton(tr("Cancel"));

    nameEdit = new QLineEdit();

    nameLabel = new QLabel(tr("Filter name: "));

    filterList = new QListWidget();

    // callbacks
    QObject::connect(addButton, SIGNAL(clicked()), SLOT(AddFilter()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));

    layout = new QGridLayout();
    layout->addWidget(filterList, 0, 0, 1, 3);
    layout->addWidget(nameLabel, 1, 0);
    layout->addWidget(nameEdit, 1, 1, 1, 2);
    layout->addWidget(addButton, 2, 0);
    layout->addWidget(cancelButton, 2, 2);

    setLayout(layout);

    setWindowTitle(tr("Add Filter"));
}

/*
    Slots
*/
void AddFilterDialog::AddFilter()
{
    if (activeView != nullptr) {
        boost::shared_ptr<insitu::Filter> fl = filterLoader->loadFilter(
            filterList->currentItem()->text().toStdString(),
            nameEdit->text().toStdString()
        );

        activeView->addFilter(fl);

        activeView = nullptr; // reset so we don't segfault on a deleted view
        accept();
    } else {
        // TODO err
        reject();
    }
}

/*
    Public Functions
*/
void AddFilterDialog::open()
{
    refreshFilters();
    QDialog::open();
}

void AddFilterDialog::setActiveView(FilteredView * view)
{
    activeView = view;
}

/*
    Private Functions
*/
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
