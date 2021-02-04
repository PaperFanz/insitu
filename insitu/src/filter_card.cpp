#include "filter_card.hpp"

namespace insitu {

/*
    Constructor/Destructor
*/
FilterCard::FilterCard(std::string name_, QWidget * parent) : QWidget(parent)
{
    name = name_;

    // ui elements
    nameLabel = new QLabel(tr(name_.c_str()));

    editButton = new QPushButton(tr("Edit"));
    upButton = new QPushButton(tr("Up"));
    downButton = new QPushButton(tr("Down"));

    // callbacks

    // layout
    layout = new QGridLayout();
    layout->addWidget(nameLabel, 0, 0, 1, 2);
    layout->addWidget(editButton, 1, 0);
    layout->addWidget(upButton, 0, 2);
    layout->addWidget(downButton, 1, 2);
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
