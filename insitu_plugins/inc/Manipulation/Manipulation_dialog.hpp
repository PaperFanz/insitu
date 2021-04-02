#ifndef insitu_plugins_Manipulation_DIALOG_HPP
#define insitu_plugins_Manipulation_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins
{
class ManipulationDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    //TODO: Apply gaussian blur to canvas background via dialog box

    QLabel* blurLabel;
    QSlider* blurSlider;
    QPushButton* okButton;
    QPushButton* cancelButton;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    ManipulationDialog(insitu::Filter* parent_);
};

}    

#endif   
