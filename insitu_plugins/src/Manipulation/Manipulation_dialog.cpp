#include <Manipulation/Manipulation_dialog.hpp>

namespace insitu_plugins
{
ManipulationDialog::ManipulationDialog(insitu::Filter* parent_)
    : FilterDialog(parent_)
{
    okButton = new QPushButton(tr("OK"));
    okButton->setDefault(true);
    cancelButton = new QPushButton(tr("Cancel"));

    blurSlider = new QSlider(Qt::Horizontal);

    blurSlider->setRange(0, 255);

    blurLabel = new QLabel(tr("Blur: "));

    layout = new QGridLayout();
    layout->addWidget(blurLabel, 0, 0);
    layout->addWidget(blurSlider, 0, 1);
    layout->addWidget(cancelButton, 1, 0);
    layout->addWidget(okButton, 1, 1);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void ManipulationDialog::onOK(void)
{
    Json::Value& settings = parent->getSettingsValue();

    settings["blur"] = blurSlider->value();

    accept();
}

}    // end namespace insitu_plugins
