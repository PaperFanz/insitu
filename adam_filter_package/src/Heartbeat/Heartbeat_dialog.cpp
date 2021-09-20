#include <Heartbeat/Heartbeat_dialog.hpp>

namespace adam_filter_package
{
HeartbeatDialog::HeartbeatDialog(insitu::Filter* parent_)
    : FilterDialog(parent_)
{
    okButton = new QPushButton(tr("OK"));
    okButton->setDefault(true);
    cancelButton = new QPushButton(tr("Cancel"));

    layout = new QGridLayout();
    layout->addWidget(cancelButton, 0, 0);
    layout->addWidget(okButton, 0, 1);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void HeartbeatDialog::onOK(void)
{
    Json::Value& settings = parent->getSettingsValue();
    // TODO change parent settings e.g. settings["key"] = value

    accept();
}

}    // end namespace adam_filter_package
