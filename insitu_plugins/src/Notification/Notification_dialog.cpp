#include <Notification/Notification_dialog.hpp>
#include <Notification/Notification.hpp>

using namespace notification_filter;

namespace insitu_plugins
{
NotificationDialog::NotificationDialog(insitu::Filter * parent_)
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

    queueBox = new QDoubleSpinBox();
    queueBox->setRange(1, 8);
    queueBox->setSingleStep(1);
    queueBox->setDecimals(0);
    queueBox->setValue(DEFAULT_QUEUE_SIZE);
    queueLabel = new QLabel(tr("Number of Messages"));
    layout->addWidget(queueLabel, 2, 0);
    layout->addWidget(queueBox, 2, 1);

    dirBox = new QCheckBox(tr("New Messages at the Bottom"), this);
    dirBox->setChecked(true);
    layout->addWidget(dirBox, 3, 0, 1, 2);

    cancelButton = new QPushButton(tr("Cancel"));
    layout->addWidget(cancelButton, 4, 0);

    okButton = new QPushButton(tr("OK"));
    okButton->setDefault(true);
    layout->addWidget(okButton, 4, 1);

    setLayout(layout);

    QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
    QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
 }

void NotificationDialog::onOK(void)
{
    Json::Value & settings = parent->getSettingsValue();

    settings["name"] = nameEdit->text().toStdString();

    static_cast<insitu_plugins::Notification*>(parent)->onTopicChange(
        topicEdit->text().toStdString());
    settings["topic"] = topicEdit->text().toStdString();

    static_cast<insitu_plugins::Notification*>(parent)->onQueueChange(
        (unsigned int)queueBox->value());
    settings["queue_size"] = (unsigned int)queueBox->value();

    static_cast<insitu_plugins::Notification*>(parent)->onDirectionChange(
        dirBox->isChecked());
    settings["msg_direction_down"] = dirBox->isChecked();

    accept();
}

} // end namespace insitu_plugins
