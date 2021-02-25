#include <insitu_plugins/crosshair_dialog.hpp>

namespace insitu_plugins {

CrosshairDialog::CrosshairDialog(insitu::Filter * parent_)
: FilterDialog(parent_)
{
    x = 0;
    y = 0;

    sizeLabel = new QLabel(tr("Size: "));
    xLabel = new QLabel(tr("X: "));
    yLabel = new QLabel(tr("Y: "));

    sizeEdit = new QLineEdit(tr("5"));
    xEdit = new QLineEdit(tr("320"));
    yEdit = new QLineEdit(tr("240"));

    okButton = new QPushButton(tr("OK"));
    cancelButton = new QPushButton(tr("Cancel"));

    layout = new QGridLayout();
    layout->addWidget(sizeLabel, 0, 0);
    layout->addWidget(sizeEdit, 0, 1);
    layout->addWidget(xLabel, 1, 0);
    layout->addWidget(xEdit, 1, 1);
    layout->addWidget(yLabel, 2, 0);
    layout->addWidget(yEdit, 2, 1);
    layout->addWidget(cancelButton, 3, 0);
    layout->addWidget(okButton, 3, 1);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void CrosshairDialog::onOK(void)
{
    parent->set("size", sizeEdit->text().toStdString());
    parent->set("x", xEdit->text().toStdString());
    parent->set("y", yEdit->text().toStdString());
    accept();
}

} // namespace insitu_plugins