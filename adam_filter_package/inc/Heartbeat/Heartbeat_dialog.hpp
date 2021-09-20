#ifndef adam_filter_package_Heartbeat_DIALOG_HPP
#define adam_filter_package_Heartbeat_DIALOG_HPP

#include <insitu/filter.hpp>

namespace adam_filter_package
{
class HeartbeatDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QPushButton* okButton;
    QPushButton* cancelButton;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    HeartbeatDialog(insitu::Filter* parent_);
};

}    // end namespace adam_filter_package

#endif    // end adam_filter_package_Heartbeat_DIALOG_HPP
