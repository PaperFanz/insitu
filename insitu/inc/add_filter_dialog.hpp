#ifndef insitu_ADD_FILTER_DIALOG_HPP
#define insitu_ADD_FILTER_DIALOG_HPP

// QT includes
#include <QtWidgets>

// plugin includes
#include "filter_factory.hpp"
#include "filtered_view.hpp"

namespace insitu
{
class AddFilterDialog : public QDialog
{
    Q_OBJECT
private:
    // ui elements
    QTreeWidget* filterTree;
    QLineEdit* nameEdit;
    QLabel* nameLabel;
    QPushButton* addBtn;
    QPushButton* cancelBtn;
    QErrorMessage* errMsg;

    // loader
    FilterFactory* filterLoader;

    // load destination
    FilteredView* activeView;

public Q_SLOTS:
    void AddFilter(void);

    void onFilterChanged(void);

public:
    AddFilterDialog(QWidget* parent = nullptr);

    ~AddFilterDialog(void);

    void setActiveView(FilteredView* view);

    bool unloadFilter(const std::string& name);

    void open(void);

private:
    void refreshFilters(void);
};

}    // namespace insitu
#endif
