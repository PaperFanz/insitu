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

#include "add_mode_dialog.hpp"
#include "add_view_dialog.hpp"
#include "add_filter_dialog.hpp"

#include "insitu_utils.hpp"

QT_BEGIN_NAMESPACE

namespace ui
{
class main_window_design
{
public:
    // menubar
    QMenuBar* menubar;
    QMenu* menuFile;
    QMenu* menuEdit;
    QMenu* menuHelp;
    QMenu* menuDocs;

    // menu options
    QAction* actionQuit;
    QAction* actionSave;
    QAction* actionLoad;
    QAction* actionRecent;
    QAction* actionNewMode;
    QAction* actionNewView;
    QAction* actionNewFilter;
    QAction* actionPreferences;
    QAction* actionROSWiki;
    QAction* actionReadme;
    QAction* actionGithub;
    QAction* actionWebsite;
    QAction* actionAbout;

    // dialog boxes
    insitu::add_mode_dialog* add_mode_dialog;
    insitu::add_view_dialog* add_view_dialog;
    insitu::add_filter_dialog* add_filter_dialog;

    // tab / mode interface
    QTabWidget* tabmanager;

    // layout containers
    QWidget* body;
    QHBoxLayout* hbox;

    // ui functions
    void setupUI(QMainWindow* mainwindow);

private:
    // ui helpers
    void setupShortcuts(QMainWindow* mainwindow);

};    // class main_window_design

}    // namespace ui

QT_END_NAMESPACE

#endif