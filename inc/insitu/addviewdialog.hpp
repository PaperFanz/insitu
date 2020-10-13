#ifndef insitu_ADD_VIEW_DIALOG_HPP
#define insitu_ADD_VIEW_DIALOG_HPP

// QT includes
#include <QDialog>
#include <QStringList>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QMdiArea>
#include <QtWidgets/QTextEdit>

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// insitu includes
#include "view.hpp"

// C++ includes
#include <vector>

namespace insitu {

class AddViewDialog : public QDialog
{

    Q_OBJECT
    private:
        QLineEdit * nameEdit;
        QComboBox * modeBox;
        QComboBox * topicBox;
        QPushButton * createButton;
        QPushButton * cancelButton;

        QFormLayout * form;
        QHBoxLayout * buttonHBox;

        QTabWidget * tabmanager;

        QStringList getModeList();
        QStringList getTopicList();
    
    public Q_SLOTS:
        void AddView(void);

    public:
        AddViewDialog(QWidget * parent = nullptr);

        void open();

};

} // namespace insitu
#endif
