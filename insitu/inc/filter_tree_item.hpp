#ifndef insitu_FILTER_TREE_ITEM_HPP
#define insitu_FILTER_TREE_ITEM_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "insitu_utils.hpp"

namespace insitu
{
class FilterTreeItem : public QTreeWidgetItem
{
private:
    std::string name;

public:
    FilterTreeItem(std::string name_, std::string type_, 
            std::string description_, QTreeWidgetItem* parent);

    const std::string& getFilterName(void) const;
};

}    // end namespace insitu

#endif
