#ifndef insitu_ADD_FILTER_DIALOG_HPP
#define insitu_ADD_FILTER_DIALOG_HPP

// QT includes
#include <QtWidgets>

// plugin includes
#include "filter_factory.hpp"
#include "filtered_view.hpp"
#include "filter_card.hpp"

namespace insitu {

class AddFilterDialog : public QDialog
{

Q_OBJECT
private:
    // ui elements
    QListWidget * filterList;
    QScrollArea * listScroll;
    QLineEdit * nameEdit;
    QLabel * nameLabel;
    QPushButton * addButton;
    QPushButton * cancelButton;

    // layout
    QGridLayout * layout;

    // loader
    FilterFactory * filterLoader;

    // load destination
    FilteredView * activeView;
    
public Q_SLOTS:
    void AddFilter(void);

public:
    AddFilterDialog(QWidget * parent = nullptr);

    void setActiveView(FilteredView * view);

    void open();

private:
    void refreshFilters(void);

};

} // namespace insitu
#endif
