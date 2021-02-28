#ifndef insitu_ADD_MODE_DIALOG_HPP
#define insitu_ADD_MODE_DIALOG_HPP

#include <QtWidgets>

#include "insitu_utils.hpp"
#include "mode_container.hpp"

namespace insitu {

class AddModeDialog : public QDialog {

  Q_OBJECT
private:
  QLineEdit *nameEdit;
  QPushButton *createButton;
  QPushButton *cancelButton;

  QHBoxLayout *hbox;
  QFormLayout *form;

  QTabWidget *tabmanager;

public Q_SLOTS:
  void AddMode(void);

public:
  AddModeDialog(QWidget *parent);
};

} // namespace insitu
#endif
