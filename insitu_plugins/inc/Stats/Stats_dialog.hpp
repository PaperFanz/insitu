#ifndef insitu_plugins_Stats_DIALOG_HPP
#define insitu_plugins_Stats_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins
{
class StatsDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QCheckBox* cpuCBox;
    QCheckBox* memCBox;
    QCheckBox* diskCBox;
    QColorDialog* colorDiag;
    QPushButton* colorBtn;

    QPushButton* okButton;
    QPushButton* cancelButton;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

    void onColor(void);

public:
    StatsDialog(insitu::Filter* parent_);
};

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Stats_DIALOG_HPP
