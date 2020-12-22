#include <QtGui>
#include <QApplication>
#include "main_window.hpp"

int main(int argc, char **argv)
{
	/* ROS INITIALIZATION */
	ros::init(argc, argv, "insitu");

    /* GUI INITIALIZATION */
    QApplication app(argc, argv);

    insitu::MainWindow w(argc,argv);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
