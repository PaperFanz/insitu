#ifndef insitu_plugins_Manip_boundary_DIALOG_HPP
#define insitu_plugins_Manip_boundary_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins
{
class ManipBoundaryDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QLabel* doubleLabel;
    QDoubleSpinBox* doubleBox;
    QLabel* redLabel;
    QSlider* redSlider;
    QLabel* greenLabel;
    QSlider* greenSlider;
    QLabel* blueLabel;
    QSlider* blueSlider;
    QPushButton* okButton;
    QPushButton* cancelButton;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    ManipBoundaryDialog(insitu::Filter* parent_);
};

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Transparent_DIALOG_HPP
