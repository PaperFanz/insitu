#ifndef insitu_ROS_IMAGE_VIEW_HPP
#define insitu_ROS_IMAGE_VIEW_HPP

#include <QtWidgets>

namespace insitu {

class RosImageFrame : public QFrame
{

Q_OBJECT
private:

    QImage img;

    mutable QMutex img_mutex;

Q_SIGNALS:

    void delayedUpdate(void);

public:

    RosImageFrame(QWidget * parent = nullptr);

    ~RosImageFrame(void);

    void setImage(const QImage & img_);

protected:

    void paintEvent(QPaintEvent* event);

};

} // namespace insitu

#endif
