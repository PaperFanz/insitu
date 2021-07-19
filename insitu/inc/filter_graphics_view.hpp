#ifndef insitu_FILTER_GRAPHICS_VIEW_HPP
#define insitu_FILTER_GRAPHICS_VIEW_HPP

// QT includes
#include <QtWidgets>

// insitu includes
#include "filter_graphics_scene.hpp"
#include "filter_graphics_item.hpp"

namespace insitu
{
class FilterGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    FilterGraphicsView(QGraphicsScene* scene, QWidget* parent = nullptr);

    void setRootItem(FilterGraphicsItem* item);

    void fitToRoot(void);

    void setReplublishing(bool repub);

    /*
        In the current implementation, calling getImage causes the view to
        think that the scene has already been rendered, and even if update()
        is called again the viewport begins to flicker. As a workaround,
        viewport updates and interaction are disabled while the filtered view
        is republishing (and therefore calling FilterGraphicsView's getImage).
        This workaround has its own shortcomings but this is less egregious
        than the flickering.

        If someone more familiar with the QGraphicsView/Scene rendering
        pipeline were to propose a solution that would resolve the flickering
        issue during republish it would be much appreciated.
    */
    const QImage& getImage(void);

    QSize getRootSize(void) const;

public slots:

    void rootImgSizeChanged(QSize size);

signals:

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    FilterGraphicsItem* root;

    QImage imgBuf;

    bool republishing;
};

}    // namespace insitu

#endif    // insitu_FILTER_GRAPHICS_VIEW_HPP

