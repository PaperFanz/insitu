#ifndef insitu_plugins_CROSSHAIR_DIALOG_HPP
#define insitu_plugins_CROSSHAIR_DIALOG_HPP

#include <insitu/filter.hpp>

namespace insitu_plugins {

class CrosshairDialog : public insitu::FilterDialog
{
Q_OBJECT
private:
    int x;
    int y;
    int size;

    QLabel * sizeLabel;
    QLineEdit * sizeEdit;

    QLabel * xLabel;
    QLineEdit * xEdit;
    
    QLabel * yLabel;
    QLineEdit * yEdit;

    QPushButton * okButton;
    QPushButton * cancelButton;

    QGridLayout * layout;

public Q_SLOTS:

    void onOK(void);

public:
    CrosshairDialog(insitu::Filter * parent_);

};

} // namespace insitu_plugins

#endif // insitu_plugins_CROSSHAIR_HPP