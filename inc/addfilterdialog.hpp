#ifndef insitu_ADD_FILTER_DIALOG_HPP
#define insitu_ADD_FILTER_DIALOG_HPP

// QT includes
#include <QtWidgets>

namespace insitu {

class AddFilterDialog : public QDialog
{

Q_OBJECT
private:
    // ui elements
    QListWidget * filterList;
    QPushButton * addButton;
    QPushButton * cancelButton;

    // layout
    QGridLayout * layout;
    
public Q_SLOTS:
    void AddFilter(void);

public:
    AddFilterDialog(QWidget * parent = nullptr);

    void open();

};

} // namespace insitu
#endif
