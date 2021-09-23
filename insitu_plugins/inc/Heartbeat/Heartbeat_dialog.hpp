#ifndef insitu_plugins_Heartbeat_DIALOG_HPP
#define insitu_plugins_Heartbeat_DIALOG_HPP

#include <insitu/filter.hpp>

namespace heartbeat_filter
{
const std::string DEFAULT_NAME = "Heartbeat";
const std::string DEFAULT_TOPIC = "heartbeat_topic";
const std::string DEFAULT_RATE = "1.0";
}    // end namespace heartbeat_filter

namespace insitu_plugins
{
class HeartbeatDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QPushButton* okButton;
    QPushButton* cancelButton;
    QLineEdit* nameEdit;
    QLabel* nameLabel;
    QLineEdit* topicEdit;
    QLabel* topicLabel;
    QLineEdit* rateEdit;
    QLabel* rateLabel;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    HeartbeatDialog(insitu::Filter* parent_);
};

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Heartbeat_DIALOG_HPP
