#ifndef insitu_plugins_Crosshair_DIALOG_HPP
#define insitu_plugins_Crosshair_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins
{
class CrosshairDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QPushButton* okButton;
    QPushButton* cancelButton;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    CrosshairDialog(insitu::Filter* parent_);
};

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Crosshair_DIALOG_HPP
