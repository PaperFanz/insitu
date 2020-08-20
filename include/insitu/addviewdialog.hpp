#ifndef insitu_ADD_VIEW_DIALOG_HPP
#define insitu_ADD_VIEW_DIALOG_HPP

#include <QDialog>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QFormLayout>

#include "../include/insitu_common/view.hpp"

namespace insitu {

class AddViewDialog : public QDialog
{

    Q_OBJECT
    private:
        QLineEdit * nameEdit;
        QComboBox * topicBox;
        QPushButton * createButton;
        QPushButton * cancelButton;

        QFormLayout * form;
        QHBoxLayout * buttonHBox;

    public:
        AddViewDialog(QWidget * parent = nullptr);

};

} // namespace insitu
#endif
