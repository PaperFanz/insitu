#ifndef insitu_ADD_MODE_DIALOG_HPP
#define insitu_ADD_MODE_DIALOG_HPP

#include <QDialog>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QFormLayout>

#include "mode.hpp"

namespace insitu {

class AddModeDialog : public QDialog
{

Q_OBJECT
private:
    QLineEdit * nameEdit;
    QPushButton * createButton;
    QPushButton * cancelButton;

    QHBoxLayout * hbox;
    QFormLayout * form;

    QTabWidget * tabmanager;

public Q_SLOTS:
    void AddMode(void);

public:
    AddModeDialog(QWidget * parent);

};

} // namespace insitu
#endif
