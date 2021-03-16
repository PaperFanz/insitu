
#ifndef insitu_MAIN_WINDOW_H
#define insitu_MAIN_WINDOW_H

#include <QMainWindow>
#include <QInputDialog>
#include "main_window_design.hpp"

namespace insitu {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	void on_actionNewMode_triggered();
	void on_actionNewView_triggered();
	void on_actionNewFilter_triggered();
	void on_actionAbout_triggered();

private:
	ui::main_window_design ui;
};

}  // namespace insitu

#endif // insitu_MAIN_WINDOW_H
