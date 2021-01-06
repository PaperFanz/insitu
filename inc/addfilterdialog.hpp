#ifndef insitu_ADD_FILTER_DIALOG_HPP
#define insitu_ADD_FILTER_DIALOG_HPP

// QT includes
#include <QtWidgets>

// plugin includes
#include <pluginlib/class_loader.h>
#include <insitu/filter.hpp>

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

    // plugin loader
    pluginlib::ClassLoader<insitu_iface::Filter> * filterLoader;
    
public Q_SLOTS:
    void AddFilter(void);

public:
    AddFilterDialog(QWidget * parent = nullptr);

    void open();

private:
    void refreshFilters(void);

    boost::shared_ptr<insitu_iface::Filter> getInstance(QString filter);

};

} // namespace insitu
#endif
