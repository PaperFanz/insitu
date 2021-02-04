#ifndef insitu_FILTER_INFO_HPP
#define insitu_FILTER_INFO_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "insitu_utils.hpp"

namespace insitu {

class FilterInfo : public QWidget
{

Q_OBJECT
private:
    // ui elements
    QLabel * typeLabel;
    QLabel * package;
    QLabel * description;

    // layout element
    QGridLayout * layout;

    // data
    std::string name;

public Q_SLOTS:

public:
    FilterInfo(std::string name_, std::string type_, std::string package_, 
               std::string description_, QWidget * parent = nullptr);

    ~FilterInfo(void);

    const std::string& 
    getFilterName(void);

};

} // end namespace insitu

#endif
