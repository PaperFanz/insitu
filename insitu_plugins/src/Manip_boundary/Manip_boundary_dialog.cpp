#include <Manip_boundary/Manip_boundary_dialog.hpp>


namespace insitu_plugins
{
ManipBoundaryDialog::ManipBoundaryDialog(insitu::Filter* parent_)
    : FilterDialog(parent_)
{
    okButton = new QPushButton(tr("OK"));
    okButton->setDefault(true);
    cancelButton = new QPushButton(tr("Cancel"));

    areaSlider = new QSlider(Qt::Horizontal);
    areaSlider->setRange(-60,65); //arbitrary max number based on robot setup
    areaLabel = new QLabel(tr("Area: "));

    layout = new QGridLayout();
    layout->addWidget(areaLabel, 0, 0);
    layout->addWidget(areaSlider, 0, 1);
    layout->addWidget(cancelButton, 1, 0);
    layout->addWidget(okButton, 1, 1);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void ManipBoundaryDialog::onOK(void)
{
    Json::Value& settings = parent->getSettingsValue();

    settings["area"] = areaSlider->value();

    accept();
}

}    // end namespace insitu_plugins
