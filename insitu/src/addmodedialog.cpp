#include "addmodedialog.hpp"

namespace insitu {

AddModeDialog::AddModeDialog(QWidget *parent) : QDialog(parent) {
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

void AddModeDialog::AddMode() {
  QString name = nameEdit->text();

  // create new tab with entered name
  ModeContainer *mode = new ModeContainer(name);
  tabmanager->addTab(mode, name);

  addNamedWidget("mode_" + name.toStdString(), mode);

  // accept and exit modal
  accept();
}

} // namespace insitu
