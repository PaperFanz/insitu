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
        doubleBox->setSingleStep(0.1);
        doubleBox->setValue(0.5);
        doubleLabel = new QLabel(tr("Transparency: "));

        redSlider = new QSlider(Qt::Horizontal);
        greenSlider = new QSlider(Qt::Horizontal);
        blueSlider = new QSlider(Qt::Horizontal);

        redSlider->setRange(0, 255);
        greenSlider->setRange(0, 255);
        blueSlider->setRange(0, 255);

        redLabel = new QLabel(tr("R: "));
        greenLabel = new QLabel(tr("G: "));
        blueLabel = new QLabel(tr("B: "));

        layout = new QGridLayout();
        layout->addWidget(redLabel, 0, 0);
        layout->addWidget(redSlider, 0, 1);
        layout->addWidget(greenLabel, 1, 0);
        layout->addWidget(greenSlider, 1, 1);
        layout->addWidget(blueLabel, 2, 0);
        layout->addWidget(blueSlider, 2, 1);
        layout->addWidget(doubleLabel, 3, 0);
        layout->addWidget(doubleBox, 3, 1);
        layout->addWidget(cancelButton, 4, 0);
        layout->addWidget(okButton, 4, 1);

        setLayout(layout);

        QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
        QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
    }

    void TransparentDialog::onOK(void)
    {
        Json::Value & settings = parent->getSettingsValue();
        // TODO change parent settings e.g. settings["key"] = value

        settings["alpha"] = doubleBox->value();
        settings["red"] = redSlider->value();
        settings["green"] = greenSlider->value();
        settings["blue"] = blueSlider->value();
        
        accept();
    }

    } // end namespace insitu_plugins
    