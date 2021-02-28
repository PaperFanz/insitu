
#include "main_window.hpp"
#include "addmodedialog.hpp"
#include <QMessageBox>
#include <QtGui>
#include <iostream>

namespace insitu {

using namespace Qt;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent) {
  ui.setupUI(this); // Calling this incidentally connects all ui's triggers to
                    // on_...() callbacks in this class.

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tabmanager->setCurrentIndex(
      0); // ensure the first tab is showing - qt-designer should have this
          // already hardwired, but often loses it (settings?).
}

MainWindow::~MainWindow() {
  // TODO
}

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(
      this, tr("About ..."),
      tr("<h2>insitu</h2><p>situational awareness TODO DESC</p>"));
}

void MainWindow::on_actionNewMode_triggered() { ui.addmodedialog->open(); }

void MainWindow::on_actionNewView_triggered() { ui.addviewdialog->open(); }

void MainWindow::on_actionNewFilter_triggered() { ui.addfilterdialog->open(); }

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "insitu");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "insitu");

  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

} // namespace insitu
