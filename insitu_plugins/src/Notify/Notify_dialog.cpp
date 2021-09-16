#include <Notify/Notify_dialog.hpp>

namespace insitu_plugins {

NotifyDialog::NotifyDialog(insitu::Filter * parent_)
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

void NotifyDialog::onOK(void)
{
    Json::Value & settings = parent->getSettingsValue();
    // TODO change parent settings e.g. settings["key"] = value
        
    accept();
}

} // end namespace insitu_plugins

