#include <Crosshair/Crosshair_dialog.hpp>

namespace insitu_plugins
{
CrosshairDialog::CrosshairDialog(insitu::Filter* parent_)
    : FilterDialog(parent_)
{
    okButton = new QPushButton(tr("OK"));
    cancelButton = new QPushButton(tr("Cancel"));

    layout = new QGridLayout();
    layout->addWidget(okButton);
    layout->addWidget(cancelButton);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void CrosshairDialog::onOK(void)
{
    // TODO change parent settings e.g.
    // parent->set("key", "setting");

    accept();
}

}    // end namespace insitu_plugins
