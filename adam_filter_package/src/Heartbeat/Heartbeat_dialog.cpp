#include <Heartbeat/Heartbeat_dialog.hpp>

namespace adam_filter_package
{
HeartbeatDialog::HeartbeatDialog(insitu::Filter* parent_)
    : FilterDialog(parent_)
{
    layout = new QGridLayout();

    nameEdit = new QLineEdit();
    nameLabel = new QLabel(tr("Name: "), nameEdit);
    layout->addWidget(nameLabel, 0, 0);
    layout->addWidget(nameEdit, 0, 1);

    topicEdit = new QLineEdit();
    topicLabel = new QLabel(tr("Topic: "), topicEdit);
    layout->addWidget(topicLabel, 1, 0);
    layout->addWidget(topicEdit, 1, 1);

    rateEdit = new QLineEdit();
    rateLabel = new QLabel(tr("Expected Rate: "), rateEdit);
    layout->addWidget(rateLabel, 2, 0);
    layout->addWidget(rateEdit, 2, 1);

    okButton = new QPushButton(tr("OK"));
    okButton->setDefault(true);
    cancelButton = new QPushButton(tr("Cancel"));
    layout->addWidget(cancelButton, 3, 0);
    layout->addWidget(okButton, 3, 1);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
}

void HeartbeatDialog::onOK(void)
{
    Json::Value& settings = parent->getSettingsValue();

    settings["name"] = nameEdit->text().toStdString();
    settings["topic"] = topicEdit->text().toStdString();
    settings["rate"] = rateEdit->text().toStdString();

    accept();
}

}    // end namespace adam_filter_package
