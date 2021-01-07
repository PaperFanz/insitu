#include "addmodedialog.hpp"

namespace insitu {

AddModeDialog::AddModeDialog(QWidget * parent) : QDialog(parent)
{
    tabmanager = (QTabWidget *)getNamedWidget("tabmanager");
    
    // mode name text input
    nameEdit = new QLineEdit();

    // modal buttons
    cancelButton = new QPushButton(tr("Cancel"));
    createButton = new QPushButton(tr("Create"));
    createButton->setDefault(true);

    // callbacks
    QObject::connect(createButton, SIGNAL(clicked()), SLOT(AddMode()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));

    // layout
    hbox = new QHBoxLayout();
    hbox->addWidget(cancelButton);
    hbox->addWidget(createButton);

    form = new QFormLayout();
    form->addRow(tr("Mode Name"), nameEdit);
    form->addRow(hbox);

    setLayout(form);

    setWindowTitle(tr("Add Mode"));
}

void AddModeDialog::AddMode()
{
    // create new tab with entered name
    ModeContainer * mode = new ModeContainer();
    tabmanager->addTab(mode, nameEdit->text());

    addNamedWidget("mode_" + nameEdit->text().toStdString(), mode);

    // accept and exit modal
    accept();
}

}
