#include <insitu_plugins/crosshair_dialog.hpp>

namespace insitu_plugins {

CrosshairDialog::CrosshairDialog(insitu::Filter * parent_)
: FilterDialog(parent_)
{
    x = 0;
    y = 0;

    textEdit = new QLineEdit();

    okButton = new QPushButton(tr("OK"));
    cancelButton = new QPushButton(tr("Cancel"));

    layout = new QGridLayout();
    layout->addWidget(textEdit);
    layout->addWidget(okButton);
    layout->addWidget(cancelButton);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void CrosshairDialog::onOK(void)
{
    parent->set("size", textEdit->text().toStdString());
    accept();
}

} // namespace insitu_plugins