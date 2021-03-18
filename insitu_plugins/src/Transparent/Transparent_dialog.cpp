#include <Transparent/Transparent_dialog.hpp>

    namespace insitu_plugins {

    TransparentDialog::TransparentDialog(insitu::Filter * parent_)
    : FilterDialog(parent_)
    {
        okButton = new QPushButton(tr("OK"));
        okButton->setDefault(true);
        cancelButton = new QPushButton(tr("Cancel"));

        doubleBox = new QDoubleSpinBox();
        doubleBox->setRange(0.0, 1.0);
        doubleLabel = new QLabel(tr("Transparency: "));

        layout = new QGridLayout();
        layout->addWidget(doubleLabel, 0, 0);
        layout->addWidget(doubleBox, 0, 1);
        layout->addWidget(cancelButton, 1, 0);
        layout->addWidget(okButton, 1, 1);

        setLayout(layout);

        QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
        QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
    }

    void TransparentDialog::onOK(void)
    {
        Json::Value & settings = parent->getSettingsValue();
        // TODO change parent settings e.g. settings["key"] = value

        settings["alpha"] = doubleBox->value();
        
        accept();
    }

    } // end namespace insitu_plugins
    