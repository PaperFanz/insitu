#include "addfilterdialog.hpp"

namespace insitu {

/*
    Constructor
*/
AddFilterDialog::AddFilterDialog(QWidget * parent) : QDialog(parent)
{
    filterLoader = new FilterFactory("insitu");
    activeView = nullptr;

    addBtn = new QPushButton(tr("Add"));
    addBtn->setDefault(true);
    cancelBtn = new QPushButton(tr("Cancel"));

    nameEdit = new QLineEdit();

    nameLabel = new QLabel(tr("Filter name: "));

    filterList = new QListWidget();

    listScroll = new QScrollArea();
    listScroll->setWidget(filterList);

    errMsg = new QErrorMessage();

    // callbacks
    QObject::connect(addBtn, SIGNAL(clicked()), SLOT(AddFilter()));
    QObject::connect(cancelBtn, SIGNAL(clicked()), SLOT(reject()));
    QObject::connect(filterList, SIGNAL(itemSelectionChanged()), 
                     SLOT(onFilterChanged()));

    layout = new QGridLayout();
    layout->addWidget(filterList, 0, 0, 1, 3);
    layout->addWidget(nameLabel, 1, 0);
    layout->addWidget(nameEdit, 1, 1, 1, 2);
    layout->addWidget(addBtn, 2, 0);
    layout->addWidget(cancelBtn, 2, 2);

    setLayout(layout);

    setWindowTitle(tr("Add Filter"));
}

/*
    Slots
*/
void AddFilterDialog::AddFilter()
{
    if (activeView != nullptr) {
        if (filterList->currentItem() == nullptr) {
            // TODO err
            reject();
        }

        QListWidgetItem * item = filterList->currentItem();
        FilterInfo * fi = (FilterInfo *) filterList->itemWidget(item);

        try {
            boost::shared_ptr<insitu::Filter> fl = filterLoader->loadFilter(
                fi->getFilterName(),
                nameEdit->text().toStdString()
            );
            activeView->addFilter(fl);
            activeView = nullptr; // reset so we don't segfault on a deleted view
            accept();
        } catch (std::runtime_error e) {
            errMsg->showMessage(QString::fromStdString(e.what()));
            
        }
    } else {
        // TODO err
        reject();
    }
}

void AddFilterDialog::onFilterChanged(void)
{
    QListWidgetItem * item = filterList->currentItem();
    FilterInfo * fi = (FilterInfo *) filterList->itemWidget(item);
    QString name = QString::fromStdString(
        activeView->getViewName() + "_" + fi->getFilterType()
    );
    nameEdit->setText(name);
}

/*
    Public Functions
*/
void AddFilterDialog::open()
{
    refreshFilters();
    filterList->setCurrentRow(0);
    QDialog::open();
}

bool AddFilterDialog::unloadFilter(const std::string & name)
{
    return filterLoader->unloadFilter(name);
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

    auto classes = filterLoader->getFilterList();
    for (auto it = classes.begin(); it != classes.end(); ++it) {
        QListWidgetItem * item = new QListWidgetItem();
        FilterInfo * fi = new FilterInfo(
            *it,
            filterLoader->getName(*it),
            filterLoader->getClassPackage(*it),
            filterLoader->getClassDescription(*it)
        );
        item->setSizeHint(fi->sizeHint());

        filterList->addItem(item);
        filterList->setItemWidget(item, fi);
        //qDebug("%s", it->c_str());
    }
}

} // END NAMESPACE INSITU
