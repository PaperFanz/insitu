#include "filter_tree_item.hpp"

namespace insitu
{
/*
    Constructor/Destructor
*/
FilterTreeItem::FilterTreeItem(std::string name_, std::string type_,
                               std::string description_,
                               QTreeWidgetItem* parent)
    : QTreeWidgetItem(parent)
{
    name = name_;
    setText(1, QString::fromStdString(type_));
    setText(2, QString::fromStdString(description_));
}

/*
    Public Functions
*/
const std::string& FilterTreeItem::getFilterName(void) const
{
    return name;
}

}    // namespace insitu
