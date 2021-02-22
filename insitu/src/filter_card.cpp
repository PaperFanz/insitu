#include "filter_card.hpp"

namespace insitu {

/*
    Constructor/Destructor
*/
FilterCard::FilterCard(std::string name_, 
    boost::shared_ptr<insitu::Filter> filter_, 
    QWidget * parent) : QWidget(parent)
{
    name = name_;

    filter = filter_;

    // ui elements
    nameLabel = new QLabel(tr(name_.c_str()));

    editButton = new QPushButton(tr("Edit"));
    if (!filter->hasSettingEditor()) {
        editButton->setDisabled(true);
    }

    // layout
    layout = new QGridLayout();
    layout->addWidget(nameLabel, 0, 0);
    layout->addWidget(editButton, 0, 1);
    layout->setSizeConstraint(QLayout::SetFixedSize);

    setLayout(layout);

    // callbacks
    QObject::connect(editButton, SIGNAL(clicked()), SLOT(showSettingsEditor()));
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

/*
    Slot Functions
*/

void FilterCard::showSettingsEditor(void)
{
    filter->openSettingEditor();
}

}
