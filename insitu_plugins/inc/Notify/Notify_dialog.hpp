#ifndef insitu_plugins_Notify_DIALOG_HPP
#define insitu_plugins_Notify_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins
{
class NotifyDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QPushButton* okButton;
    QPushButton* cancelButton;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    NotifyDialog(insitu::Filter* parent_);
};

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Notify_DIALOG_HPP
