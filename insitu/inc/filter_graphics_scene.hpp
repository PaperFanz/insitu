#ifndef insitu_FILTER_GRAPHICS_SCENE_HPP
#define insitu_FILTER_GRAPHICS_SCENE_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "filter_graphics_item.hpp"

namespace insitu {

class FilterGraphicsScene : public QGraphicsScene
{
Q_OBJECT
public:

    FilterGraphicsScene(QObject * parent = nullptr);

public slots:

signals:

protected:

private:

};

} // namespace insitu

#endif // insitu_FILTER_GRAPHICS_SCENE_HPP
