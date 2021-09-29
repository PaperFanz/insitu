#include "add_filter_dialog.hpp"
#include "filter_tree_item.hpp"

namespace insitu
{
/*
    Constructor
*/
AddFilterDialog::AddFilterDialog(QWidget* parent) : QDialog(parent)
{
    filterLoader = new FilterFactory();
    activeView = nullptr;

    addBtn = new QPushButton(tr("Add"));
    addBtn->setDefault(true);
    cancelBtn = new QPushButton(tr("Cancel"));

    nameEdit = new QLineEdit();

    nameLabel = new QLabel(tr("Filter name: "));

    filterTree = new QTreeWidget();
    filterTree->setColumnCount(3);
    filterTree->setHeaderLabels({ "Package", "Type", "Description" });

    errMsg = new QErrorMessage(this);

    // callbacks
    QObject::connect(addBtn, SIGNAL(clicked()), SLOT(AddFilter()));
    QObject::connect(cancelBtn, SIGNAL(clicked()), SLOT(reject()));
    QObject::connect(filterTree, SIGNAL(itemSelectionChanged()),
                     SLOT(onFilterChanged()));

    QHBoxLayout* filterbox = new QHBoxLayout();
    filterbox->addWidget(filterTree);

    QHBoxLayout* namebox = new QHBoxLayout();
    namebox->addWidget(nameLabel);
    namebox->addWidget(nameEdit, 1);

    QHBoxLayout* buttonbox = new QHBoxLayout();
    buttonbox->addWidget(addBtn);
    buttonbox->addWidget(cancelBtn);

    QVBoxLayout* vbox = new QVBoxLayout(this);
    vbox->addLayout(filterbox, 1);
    vbox->addLayout(namebox);
    vbox->addLayout(buttonbox);

    setWindowTitle(tr("Add Filter"));
    resize(500, 500);
}

AddFilterDialog::~AddFilterDialog(void)
{
    delete filterLoader;
}

/*
    Slots
*/
void AddFilterDialog::AddFilter()
{
    if (activeView != nullptr)
    {
        FilterTreeItem* item = (FilterTreeItem*)filterTree->currentItem();
        if (item == nullptr || item->text(1).isEmpty())
        {
            errMsg->showMessage(tr("No loadable filter!"));
            reject();
        }
        else
        {
            try
            {
                auto filter = filterLoader->loadFilter(
                    item->getFilterName(), nameEdit->text().toStdString(),
                    activeView->getViewTopic());
                activeView->addFilter(filter);

                // reset so we don't segfault on a deleted view
                activeView = nullptr;

                accept();
            }
            catch (std::runtime_error e)
            {
                errMsg->showMessage(QString::fromStdString(e.what()));
            }
        }
    }
    else
    {
        errMsg->showMessage(tr("No selected view!"));
        reject();
    }
}

void AddFilterDialog::onFilterChanged(void)
{
    QString fn = filterTree->currentItem()->text(1);
    if (!fn.isEmpty())
    {
        nameEdit->setText(
            QString::fromStdString(activeView->getViewName() + "_") + fn);
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

bool AddFilterDialog::unloadFilter(const std::string& name)
{
    return filterLoader->unloadFilter(name);
}

void AddFilterDialog::setActiveView(FilteredView* view)
{
    activeView = view;
}

/*
    Private Functions
*/
void AddFilterDialog::refreshFilters(void)
{
    filterTree->clear();

    std::map<std::string, QTreeWidgetItem*> packageMap;
    auto classes = filterLoader->getFilterList();
    for (auto it = classes.begin(); it != classes.end(); ++it)
    {
        std::string name = filterLoader->getName(*it);
        std::string package = filterLoader->getClassPackage(*it);
        std::string description = filterLoader->getClassDescription(*it);

        auto pkg = packageMap.find(package);
        QTreeWidgetItem* pf;
        if (pkg == packageMap.end())
        {
            pf = new QTreeWidgetItem(filterTree);
            pf->setText(0, QString::fromStdString(package));
            pf->setExpanded(true);
            packageMap.insert({ package, pf });
        }
        else
        {
            pf = pkg->second;
        }
        FilterTreeItem* fi = new FilterTreeItem(*it, name, description, pf);
    }

    for (int i = 0; i < filterTree->columnCount(); ++i)
    {
        filterTree->resizeColumnToContents(i);
    }
}

}    // namespace insitu
