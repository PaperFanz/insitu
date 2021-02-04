#ifndef insitu_FILTER_CARD_HPP
#define insitu_FILTER_CARD_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "insitu_utils.hpp"

namespace insitu {

class FilterCard : public QWidget
{

Q_OBJECT
private:
    // ui elements
    QLabel * nameLabel;

    QPushButton * editButton;
    QPushButton * upButton;
    QPushButton * downButton;

    // layout element
    QGridLayout * layout;

    // data
    std::string name;

public Q_SLOTS:

public:
    FilterCard(std::string name_, QWidget * parent = nullptr);

    ~FilterCard(void);

    const std::string& getFilterName(void);

};

} // end namespace insitu

#endif
