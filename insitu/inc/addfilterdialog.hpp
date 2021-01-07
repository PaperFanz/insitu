#ifndef insitu_ADD_FILTER_DIALOG_HPP
#define insitu_ADD_FILTER_DIALOG_HPP

// QT includes
#include <QtWidgets>

// plugin includes
#include "filter_factory.hpp"

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

    // loader
    FilterFactory * filterLoader;
    
public Q_SLOTS:
    void AddFilter(void);

public:
    AddFilterDialog(QWidget * parent = nullptr);

    void open();

private:
    void refreshFilters(void);

};

} // namespace insitu
#endif
