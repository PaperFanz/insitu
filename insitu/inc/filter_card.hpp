#ifndef insitu_FILTER_CARD_HPP
#define insitu_FILTER_CARD_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "insitu_utils.hpp"
#include <insitu/filter.hpp>

namespace insitu {

class FilterCard : public QWidget {

  Q_OBJECT
private:
  // ui elements
  QLabel *nameLabel;

  QPushButton *editButton;

  // layout element
  QGridLayout *layout;

  // data
  std::string name;

  boost::shared_ptr<insitu::Filter> filter;

public Q_SLOTS:

  void showSettingsEditor(void);

public:
  FilterCard(std::string name_, boost::shared_ptr<insitu::Filter> filter_,
             QWidget *parent = nullptr);

  ~FilterCard(void);

  const std::string &getFilterName(void);

private:
};

} // end namespace insitu

#endif
