#include <QtGui>
#include <QApplication>
#include "main_window.hpp"
#include "insitu_utils.hpp"

int main(int argc, char **argv)
{
	/* ROS INITIALIZATION */
	ros::init(argc, argv, "insitu");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    insitu::setNodeHandle(nh);

    /* GUI INITIALIZATION */
    QApplication app(argc, argv);

    insitu::MainWindow w(argc,argv);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    spinner.stop();
    ros::shutdown();

	return result;
}
