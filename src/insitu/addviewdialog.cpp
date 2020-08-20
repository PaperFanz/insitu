#include "include/insitu/addviewdialog.hpp"

namespace insitu {

AddViewDialog::AddViewDialog(QWidget * parent) : QDialog(parent)
{
    // View name text input
    nameEdit = new QLineEdit;

    // Topic name selector
    topicBox = new QComboBox();

    // cancel button
    cancelButton = new QPushButton(tr("Cancel"));
    createButton = new QPushButton(tr("Create"));
    createButton->setDefault(true);

    // callbacks
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));

    // layout
    buttonHBox = new QHBoxLayout();
    buttonHBox->addWidget(cancelButton);
    buttonHBox->addWidget(createButton);

    form = new QFormLayout();
    form->addRow(tr("View Name"), nameEdit);
    form->addRow(tr("View Topic"), topicBox);
    form->addRow(buttonHBox);

    setLayout(form);

    setWindowTitle(tr("Add View"));
}

}
