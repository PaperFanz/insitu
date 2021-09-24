#include <Heartbeat/Heartbeat_dialog.hpp>
#include <Heartbeat/Heartbeat.hpp>

using namespace heartbeat_filter;

namespace insitu_plugins
{
HeartbeatDialog::HeartbeatDialog(insitu::Filter* parent_)
    : FilterDialog(parent_)
{
    layout = new QGridLayout();

    error_msg = new QErrorMessage();

    nameEdit = new QLineEdit(QString::fromStdString(DEFAULT_NAME));
    nameLabel = new QLabel(tr("Name: "), nameEdit);
    layout->addWidget(nameLabel, 0, 0);
    layout->addWidget(nameEdit, 0, 1);

    topicEdit = new QLineEdit(QString::fromStdString(DEFAULT_TOPIC));
    topicLabel = new QLabel(tr("Topic: "), topicEdit);
    layout->addWidget(topicLabel, 1, 0);
    layout->addWidget(topicEdit, 1, 1);

    rateEdit = new QLineEdit(QString::fromStdString(DEFAULT_RATE));
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

    static_cast<insitu_plugins::Heartbeat*>(parent)->onTopicChange(
        topicEdit->text().toStdString());
    settings["topic"] = topicEdit->text().toStdString();

    double expected_hz;
    try
    {
        expected_hz = std::stod(rateEdit->text().toStdString());
    }
    catch (const std::invalid_argument& e)
    {
        expected_hz = std::stod(DEFAULT_RATE);
        error_msg->showMessage(
            "Unable to read Rate value, using 1.0 as a default");
    }
    settings["rate"] = expected_hz;

    accept();
}

}    // end namespace insitu_plugins
