/*
    declares insitu's main window UI
*/

#ifndef INSITU_MAIN_WINDOW_DESIGN_HPP
#define INSITU_MAIN_WINDOW_DESIGN_HPP

#include <QtCore/QLocale>

#include <QtWidgets/QAction>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QApplication>

#include "addmodedialog.hpp"
#include "addviewdialog.hpp"
#include "addfilterdialog.hpp"

#include "insitu_utils.hpp"

QT_BEGIN_NAMESPACE

namespace ui {

class MainWindowDesign {

public:

    // menubar
    QMenuBar * menubar;
    QMenu * menuFile;
    QMenu * menuEdit;
    QMenu * menuHelp;
    QMenu * menuDocs;

    // menu options
    QAction * actionQuit;
    QAction * actionSave;
    QAction * actionLoad;
    QAction * actionRecent;
    QAction * actionNewMode;
    QAction * actionNewView;
    QAction * actionNewFilter;
    QAction * actionPreferences;
    QAction * actionROSWiki;
    QAction * actionReadme;
    QAction * actionGithub;
    QAction * actionWebsite;
    QAction * actionAbout;

    // dialog boxes
    insitu::AddModeDialog * addmodedialog;
    insitu::AddViewDialog * addviewdialog;
    insitu::AddFilterDialog * addfilterdialog;

    // tab / mode interface
    QTabWidget * tabmanager;

    // layout containers
    QWidget * body;
    QHBoxLayout * hbox;

    // ui functions
    void setupUI(QMainWindow * mainwindow);

private:

    // ui helpers
    void setupShortcuts(QMainWindow * mainwindow);

}; // class MainWindowDesign

} // namespace ui

QT_END_NAMESPACE

#endif