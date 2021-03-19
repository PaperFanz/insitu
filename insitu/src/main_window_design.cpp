#include "main_window_design.hpp"

namespace ui
{
void main_window_design::setupUI(QMainWindow* mainwindow)
{
    // set upject name for find() indexing
    if (mainwindow->objectName().isEmpty())
    {
        mainwindow->setObjectName(QStringLiteral("MainWindow"));
    }
    // app locale
    mainwindow->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
    mainwindow->resize(900, 600);

    insitu::addNamedWidget("root", mainwindow);

    // declare menu actions
    actionQuit = new QAction(mainwindow);
    actionQuit->setObjectName(QStringLiteral("actionQuit"));
    actionQuit->setText(QStringLiteral("Quit"));
    actionSave = new QAction(mainwindow);
    actionSave->setObjectName(QStringLiteral("actionSave"));
    actionSave->setText(QStringLiteral("Save"));
    actionLoad = new QAction(mainwindow);
    actionLoad->setObjectName(QStringLiteral("actionLoad"));
    actionLoad->setText(QStringLiteral("Load"));
    actionRecent = new QAction(mainwindow);
    actionRecent->setObjectName(QStringLiteral("actionRecent"));
    actionRecent->setText(QStringLiteral("Recent"));
    actionNewMode = new QAction(mainwindow);
    actionNewMode->setObjectName(QStringLiteral("actionNewMode"));
    actionNewMode->setText(QStringLiteral("New Mode"));
    actionNewView = new QAction(mainwindow);
    actionNewView->setObjectName(QStringLiteral("actionNewView"));
    actionNewView->setText(QStringLiteral("New View"));
    actionNewFilter = new QAction(mainwindow);
    actionNewFilter->setObjectName(QStringLiteral("actionNewFilter"));
    actionNewFilter->setText(QStringLiteral("New Filter"));
    actionPreferences = new QAction(mainwindow);
    actionPreferences->setObjectName(QStringLiteral("actionPreferences"));
    actionPreferences->setText(QStringLiteral("Preferences"));
    actionROSWiki = new QAction(mainwindow);
    actionROSWiki->setObjectName(QStringLiteral("actionROSWiki"));
    actionROSWiki->setText(QStringLiteral("ROS Wiki"));
    actionReadme = new QAction(mainwindow);
    actionReadme->setObjectName(QStringLiteral("actionReadme"));
    actionReadme->setText(QStringLiteral("Readme"));
    actionGithub = new QAction(mainwindow);
    actionGithub->setObjectName(QStringLiteral("actionGithub"));
    actionGithub->setText(QStringLiteral("Github"));
    actionWebsite = new QAction(mainwindow);
    actionWebsite->setObjectName(QStringLiteral("actionWebsite"));
    actionWebsite->setText(QStringLiteral("Website"));
    actionAbout = new QAction(mainwindow);
    actionAbout->setObjectName(QStringLiteral("actionAbout"));
    actionAbout->setText(QStringLiteral("About"));

    // declare layout widgets
    body = new QWidget(mainwindow);
    body->setObjectName(QStringLiteral("body"));
    hbox = new QHBoxLayout(body);
    hbox->setObjectName(QStringLiteral("hbox"));

    // main widget (modes represented in tabs)
    tabmanager = new QTabWidget(body);
    tabmanager->setObjectName(QStringLiteral("tabmanager"));
    tabmanager->setMinimumSize(QSize(100, 0));
    tabmanager->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
    tabmanager->setMovable(true);
    insitu::addNamedWidget("tabmanager", tabmanager);
    hbox->addWidget(tabmanager);

    // dialog widgets
    add_mode_dialog = new insitu::add_mode_dialog(mainwindow);
    add_mode_dialog->setObjectName(QStringLiteral("add_mode_dialog"));
    add_view_dialog = new insitu::add_view_dialog(mainwindow);
    add_view_dialog->setObjectName(QStringLiteral("add_view_dialog"));
    add_filter_dialog = new insitu::add_filter_dialog(mainwindow);
    add_filter_dialog->setObjectName(QStringLiteral("add_filter_dialog"));
    insitu::addNamedWidget("add_filter_dialog", add_filter_dialog);

    mainwindow->setCentralWidget(body);

    // menubar widget
    menubar = new QMenuBar(mainwindow);
    menubar->setObjectName(QStringLiteral("menubar"));
    menubar->setGeometry(QRect(0, 0, 900, 20));
    menuFile = new QMenu(menubar);
    menuFile->setObjectName(QStringLiteral("menuFile"));
    menuFile->setTitle(QStringLiteral("File"));
    menuEdit = new QMenu(menubar);
    menuEdit->setObjectName(QStringLiteral("menuEdit"));
    menuEdit->setTitle(QStringLiteral("Edit"));
    menuHelp = new QMenu(menubar);
    menuHelp->setObjectName(QStringLiteral("menuHelp"));
    menuHelp->setTitle(QStringLiteral("Help"));
    menuDocs = new QMenu(menuHelp);
    menuDocs->setObjectName(QStringLiteral("menuDocs"));
    menuDocs->setTitle(QStringLiteral("Docs"));

    menubar->addAction(menuFile->menuAction());
    menubar->addAction(menuEdit->menuAction());
    menubar->addAction(menuHelp->menuAction());
    menuFile->addAction(actionSave);
    menuFile->addAction(actionLoad);
    menuFile->addAction(actionRecent);
    menuFile->addSeparator();
    menuFile->addAction(actionQuit);
    menuEdit->addAction(actionNewMode);
    menuEdit->addAction(actionNewView);
    menuEdit->addAction(actionNewFilter);
    menuEdit->addSeparator();
    menuEdit->addAction(actionPreferences);
    menuHelp->addAction(menuDocs->menuAction());
    menuHelp->addAction(actionAbout);
    menuDocs->addAction(actionReadme);
    menuDocs->addAction(actionGithub);
    menuDocs->addAction(actionWebsite);

    mainwindow->setMenuBar(menubar);

    setupShortcuts(mainwindow);

    QObject::connect(actionQuit, SIGNAL(triggered()), mainwindow,
                     SLOT(close()));
    QMetaObject::connectSlotsByName(mainwindow);
}

void main_window_design::setupShortcuts(QMainWindow* mainwindow)
{
#ifndef QT_NO_SHORTCUT
    actionQuit->setShortcutContext(Qt::ApplicationShortcut);
    actionSave->setShortcutContext(Qt::ApplicationShortcut);
    actionLoad->setShortcutContext(Qt::ApplicationShortcut);
    actionNewMode->setShortcutContext(Qt::ApplicationShortcut);
    actionNewView->setShortcutContext(Qt::ApplicationShortcut);
    actionNewFilter->setShortcutContext(Qt::ApplicationShortcut);
    actionPreferences->setShortcutContext(Qt::ApplicationShortcut);

    actionQuit->setShortcut(
        QApplication::translate("MainWindow", "Ctrl+Q", Q_NULLPTR));
    actionSave->setShortcut(
        QApplication::translate("MainWindow", "Ctrl+S", Q_NULLPTR));
    actionLoad->setShortcut(
        QApplication::translate("MainWindow", "Ctrl+L", Q_NULLPTR));
    actionNewMode->setShortcut(
        QApplication::translate("MainWindow", "Ctrl+Shift+M", Q_NULLPTR));
    actionNewView->setShortcut(
        QApplication::translate("MainWindow", "Ctrl+Shift+V", Q_NULLPTR));
    actionNewFilter->setShortcut(
        QApplication::translate("MainWindow", "Ctrl+Shift+F", Q_NULLPTR));
    actionPreferences->setShortcut(
        QApplication::translate("MainWindow", "Ctrl+Shift+P", Q_NULLPTR));
#endif
}

}    // namespace ui