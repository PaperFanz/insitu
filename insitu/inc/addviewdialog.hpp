#ifndef insitu_ADD_VIEW_DIALOG_HPP
#define insitu_ADD_VIEW_DIALOG_HPP

// QT includes
#include <QtWidgets>

// ROS includes
#include <image_transport/image_transport.h>
#include <ros/master.h>
#include <ros/ros.h>

// insitu includes
#include "filtered_view.hpp"
#include "insitu_utils.hpp"
#include "mode_container.hpp"

namespace insitu {

class AddViewDialog : public QDialog {

  Q_OBJECT
private:
  QLineEdit *nameEdit;
  QComboBox *modeBox;
  QComboBox *topicBox;
  QPushButton *createButton;
  QPushButton *cancelButton;

  QFormLayout *form;
  QHBoxLayout *buttonHBox;

  QTabWidget *tabmanager;

  QList<QString> getModeList();

public Q_SLOTS:
  void AddView(void);

public:
  AddViewDialog(QWidget *parent = nullptr);

  void open();
};

} // namespace insitu
#endif
