#ifndef insitu_plugins_Notification_DIALOG_HPP
#define insitu_plugins_Notification_DIALOG_HPP

#include <insitu/filter.hpp>

namespace notification_filter
{
    const std::string DEFAULT_NAME = "Notification";
    const std::string DEFAULT_TOPIC = "notification_topic";
    const std::string DEFAULT_QUEUE_SIZE = "8";
    const unsigned int DEFAULT_QUEUE_SIZE2 = 8;
}   // end namespace notification_filter

namespace insitu_plugins
{
class NotificationDialog : public insitu::FilterDialog
{
Q_OBJECT
private:
    QGridLayout * layout;
    QErrorMessage* error_msg;

    QLineEdit* nameEdit;
    QLabel* nameLabel;
    QLineEdit* topicEdit;
    QLabel* topicLabel;
    QDoubleSpinBox* queueBox;
    QLabel* queueLabel;
    QCheckBox* dirBox;

    QPushButton * okButton;
    QPushButton * cancelButton;

public Q_SLOTS:

    void onOK(void);

public:
    NotificationDialog(insitu::Filter * parent_);
};

} // end namespace insitu_plugins

#endif // end insitu_plugins_Notification_DIALOG_HPP
