#include "addfilterdialog.hpp"

namespace insitu {

AddFilterDialog::AddFilterDialog(QWidget * parent) : QDialog(parent)
{
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
    QDialog::open();
}

} // END NAMESPACE INSITU
