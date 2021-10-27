#include "filter_card.hpp"

namespace insitu
{
/*
    Constructor/Destructor
*/
FilterCard::FilterCard(std::string name_,
                       boost::shared_ptr<insitu::Filter> filter,
                       QWidget* parent)
    : QWidget(parent)
{
    name = name_;

    filter_ = filter;

    // ui elements
    nameLabel = new QLabel(tr(name_.c_str()));

    editButton = new QPushButton();
    editButton->setIcon(QIcon(":/images/edit-solid.svg"));
    editButton->setToolTip("Edit filter settings");
    if (!filter_->hasSettingEditor())
    {
        editButton->setDisabled(true);
    }

    visibilityBox = new QCheckBox();
    visibilityBox->setStyleSheet(
        "QCheckBox::indicator {width: 18px; height: 18px;}"
        "QCheckBox::indicator:checked {image: url(:/images/eye-solid.svg);}"
        "QCheckBox::indicator:unchecked {image: "
        "url(:/images/eye-slash-solid.svg);}");
    visibilityBox->setToolTip("Toggle visibility");
    visibilityBox->setChecked(filter_->isVisible());

    // layout
    layout = new QGridLayout();
    layout->addWidget(nameLabel, 0, 0);
    layout->addWidget(editButton, 0, 1);
    layout->addWidget(visibilityBox, 0, 2);
    layout->setColumnStretch(0, 5);

    setLayout(layout);

    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    // callbacks
    connect(editButton, SIGNAL(clicked()), SLOT(showSettingsEditor()));
    connect(visibilityBox, SIGNAL(stateChanged(int)), this,
            SLOT(onVisibilityChanged(int)));
}

FilterCard::~FilterCard(void)
{
    filter_.reset();
}

/*
    Public Functions
*/

const std::string& FilterCard::getFilterName(void) const
{
    return name;
}

boost::shared_ptr<insitu::Filter> FilterCard::filter(void) const
{
    return filter_;
}

/*
    Slot Functions
*/

void FilterCard::showSettingsEditor(void)
{
    filter_->openSettingEditor();
}

void FilterCard::onVisibilityChanged(int state)
{
    emit visibilityChanged(state == Qt::Checked);
}

}    // namespace insitu
