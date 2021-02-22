#ifndef insitu_FILTER_CARD_HPP
#define insitu_FILTER_CARD_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "insitu_utils.hpp"
#include <insitu/filter.hpp>

namespace insitu {

class FilterCard : public QWidget
{

Q_OBJECT
private:
    // ui elements
    QLabel * nameLabel;

    QPushButton * editButton;

    QDialog * settingsDialog;

    // layout element
    QGridLayout * layout;

    // data
    std::string name;

public Q_SLOTS:

    void showSettingsEditor(void);

public:

    FilterCard(std::string name_, QDialog * settingsDialog_, 
        QWidget * parent = nullptr);

    ~FilterCard(void);

    const std::string& getFilterName(void);

private:


};

} // end namespace insitu

#endif
