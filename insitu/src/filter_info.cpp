#include "filter_info.hpp"

namespace insitu
{
/*
    Constructor/Destructor
*/
FilterInfo::FilterInfo(std::string name_, std::string type_,
                       std::string package_, std::string description_,
                       QWidget* parent)
    : QWidget(parent)
{
    name = name_;

    // ui elements
    typeLabel = new QLabel(tr(type_.c_str()));
    package = new QLabel(tr(package_.c_str()));
    description = new QLabel(tr(description_.c_str()));

    // callbacks

    // layout
    layout = new QGridLayout();
    layout->addWidget(typeLabel, 0, 0);
    layout->addWidget(package, 0, 2);
    layout->addWidget(description, 1, 0, 1, 3);
    layout->setColumnStretch(1, 1);
    layout->setSizeConstraint(QLayout::SetFixedSize);

    setLayout(layout);
}

FilterInfo::~FilterInfo(void)
{
}

/*
    Public Functions
*/
const std::string& FilterInfo::getFilterName(void) const
{
    return name;
}

const std::string FilterInfo::getFilterType(void) const
{
    return typeLabel->text().toStdString();
}

}    // namespace insitu
