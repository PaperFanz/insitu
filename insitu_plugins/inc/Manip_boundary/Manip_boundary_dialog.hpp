#ifndef insitu_plugins_Manip_boundary_DIALOG_HPP
#define insitu_plugins_Manip_boundary_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins
{
class ManipBoundaryDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QSlider* areaSlider;
    QLabel* areaLabel;
    QPushButton* okButton;
    QPushButton* cancelButton;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    ManipBoundaryDialog(insitu::Filter* parent_);
};

}     
#endif    