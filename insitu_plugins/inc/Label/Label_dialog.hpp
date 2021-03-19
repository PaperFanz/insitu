#ifndef insitu_plugins_Label_DIALOG_HPP
#define insitu_plugins_Label_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins
{
class LabelDialog : public insitu::FilterDialog
{
    Q_OBJECT
private:
    QPushButton* okButton;
    QPushButton* cancelButton;
    QLineEdit* textEdit;
    QLabel* textLabel;

    QGridLayout* layout;

public Q_SLOTS:

    void onOK(void);

public:
    LabelDialog(insitu::Filter* parent_);
};

}    // end namespace insitu_plugins

#endif    // end insitu_plugins_Label_DIALOG_HPP
