#include <Label/Label_dialog.hpp>

namespace insitu_plugins {

LabelDialog::LabelDialog(insitu::Filter * parent_)
: FilterDialog(parent_)
{
    okButton = new QPushButton(tr("OK"));
    okButton->setDefault(true);
    cancelButton = new QPushButton(tr("Cancel"));
    textEdit = new QLineEdit();
    textLabel = new QLabel(tr("Text: "), textEdit);

    layout = new QGridLayout();
    layout->addWidget(textLabel, 0, 0);
    layout->addWidget(textEdit, 0, 1);
    layout->addWidget(cancelButton, 1, 0);
    layout->addWidget(okButton, 1, 1);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void LabelDialog::onOK(void)
{
    Json::Value & settings = parent->getSettingsValue();
    // TODO change parent settings e.g. settings["key"] = value

    settings["text"] = textEdit->text().toStdString();
    
    accept();
}

} // end namespace insitu_plugins
