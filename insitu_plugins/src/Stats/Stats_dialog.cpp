#include <Stats/Stats_dialog.hpp>

namespace insitu_plugins
{
StatsDialog::StatsDialog(insitu::Filter* parent_) : FilterDialog(parent_)
{
    cpuCBox = new QCheckBox(tr("Show CPU Usage"), this);
    cpuCBox->setChecked(true);
    memCBox = new QCheckBox(tr("Show Memory Usage"), this);
    memCBox->setChecked(true);
    diskCBox = new QCheckBox(tr("Show Disk Usage"), this);
    diskCBox->setChecked(true);

    colorDiag = new QColorDialog(QColor(0, 0, 0), this);
    colorDiag->setOption(QColorDialog::DontUseNativeDialog, true);
    colorBtn = new QPushButton(tr("Select Text Color"), this);

    okButton = new QPushButton(tr("OK"), this);
    okButton->setDefault(true);
    cancelButton = new QPushButton(tr("Cancel"), this);

    layout = new QGridLayout(this);
    layout->addWidget(cpuCBox, 0, 0, 1, 2);
    layout->addWidget(memCBox, 1, 0, 1, 2);
    layout->addWidget(diskCBox, 2, 0, 1, 2);
    layout->addWidget(colorBtn, 3, 0, 1, 2);
    layout->addWidget(cancelButton, 4, 0);
    layout->addWidget(okButton, 4, 1);

    setLayout(layout);

    QObject::connect(colorBtn, SIGNAL(clicked()), SLOT(onColor()));
    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void StatsDialog::onColor(void)
{
    colorDiag->open();
}

void StatsDialog::onOK(void)
{
    Json::Value& settings = parent->getSettingsValue();

    settings["showCPU"] = cpuCBox->isChecked();
    settings["showMem"] = memCBox->isChecked();
    settings["showDisk"] = diskCBox->isChecked();

    QColor c = colorDiag->selectedColor();
    settings["r"] = c.red();
    settings["g"] = c.green();
    settings["b"] = c.blue();
    settings["a"] = c.alpha();

    accept();
}

}    // end namespace insitu_plugins
