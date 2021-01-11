#include "filter_card.hpp"

namespace insitu {

/*
    Constructor/Destructor
*/
FilterCard::FilterCard(std::string name_, std::string type_, 
                       std::string package_, std::string description_, 
                       QWidget * parent) : QWidget(parent)
{
    name = name_;

    // ui elements
    type = new QLabel(tr(type_.c_str()));
    package = new QLabel(tr(package_.c_str()));
    description = new QLabel(tr(description_.c_str()));

    // callbacks

    // layout
    layout = new QGridLayout();
    layout->addWidget(type, 0, 0);
    layout->addWidget(package, 0, 2);
    layout->addWidget(description, 1, 0, 1, 3);
    layout->setColumnStretch(1, 1);
    layout->setSizeConstraint(QLayout::SetFixedSize);

    setLayout(layout);
}

FilterCard::~FilterCard(void)
{

}

/*
    Public Functions
*/
const std::string& FilterCard::getFilterName(void)
{
    return name;
}

}
