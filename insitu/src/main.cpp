#include <QtGui>
#include <QApplication>
#include "main_window.hpp"
#include "insitu_utils.hpp"

int main(int argc, char** argv)
{
    /* ROS INITIALIZATION */
    ros::init(argc, argv, "insitu");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    /* GUI INITIALIZATION */
    QApplication app(argc, argv);

    std::thread kill_thread([&app] {
        ros::Rate loop_rate(4);
        while (ros::ok())
        {
            loop_rate.sleep();
        }
        app.exit();
    });

    insitu::MainWindow w(argc, argv);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    ros::shutdown();
    ros::waitForShutdown();
    if (kill_thread.joinable()) kill_thread.join();

    return result;
}
