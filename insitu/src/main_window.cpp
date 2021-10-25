
#include <qfiledialog.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "insitu_utils.hpp"
#include "main_window.hpp"
#include "add_mode_dialog.hpp"
#include "mode_container.hpp"

namespace insitu
{
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUI(this);
    setWindowIcon(QIcon(":/images/insitu-icon.png"));
    ui.tabmanager->setCurrentIndex(0);
    QObject::connect(ui.tabmanager, SIGNAL(tabCloseRequested(int)), this,
                     SLOT(modeClose(int)));

    if (argc >= 3)
    {
        std::string flag = argv[1];
        if (flag == "-f")
        {
            std::string restorepath = argv[2];
            ReadSettings(restorepath);
        }
    }
    else
    {
        /* reads stored settings and attempts to restore last saved
         * configuration */
        ReadSettings();
    }
}

MainWindow::~MainWindow()
{
}

/*
    Public Slots
*/
void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."),
                       tr("<h2>insitu</h2><p>situational awareness TODO "
                          "DESC</p>"));
}

void MainWindow::on_actionNewMode_triggered()
{
    ui.add_mode_dialog->open();
}

void MainWindow::on_actionNewView_triggered()
{
    ui.add_view_dialog->open();
}

void MainWindow::on_actionNewFilter_triggered()
{
    ui.add_filter_dialog->open();
}

void MainWindow::on_actionSave_triggered()
{
    Json::Value json;
    save(json);

    std::ofstream file;
    file.open(QFileDialog::getSaveFileName(this, tr("Save File"),
                                           QDir::currentPath(),
                                           tr("JSON (*.json)"))
                  .toStdString());
    if (file.is_open())
    {
        Json::StreamWriterBuilder sb;
        sb["indentation"] = "    ";
        auto writer = sb.newStreamWriter();
        writer->write(json, &file);
        file.close();
    }
}

void MainWindow::on_actionLoad_triggered()
{
    lastLoadedFile = QFileDialog::getOpenFileName(
        this, tr("Open File"), QDir::currentPath(), tr("JSON (*.json)"));
    restore(lastLoadedFile);
    if (!recentFiles.contains(lastLoadedFile))
    {
        recentFiles.append(lastLoadedFile);
    }
}

void MainWindow::modeClose(int index)
{
    delete ui.tabmanager->widget(index);
    ui.tabmanager->removeTab(index);
}

void MainWindow::ReadSettings(std::string restorepath)
{
    QSettings settings("Qt-Ros Package", "insitu");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    if (!restorepath.empty())
    {
        restore(QString::fromStdString(restorepath));
    }
    else if (settings.contains("loadfile"))
    {
        // qDebug("restore from last loaded file");
        restore(settings.value("loadfile").toString());
    }
    else
    {
        /* show tutorial if no file is restored */
        ModeContainer* tutorialMode = new ModeContainer(tr("Tutorial"));
        ui.tabmanager->addTab(tutorialMode, tr("Tutorial"));
    }
    if (settings.contains("recent"))
    {
        recentFiles.append(settings.value("recent").toStringList());
        ui.menuRecents->clear();
        for (int i = 0; i < recentFiles.size(); ++i)
        {
            QAction* recent = new QAction(recentFiles[i], ui.menuRecents);
            ui.menuRecents->addAction(recent);
        }
    }
}

void MainWindow::WriteSettings(std::string restorepath)
{
    QSettings settings("Qt-Ros Package", "insitu");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("recent", recentFiles);
    if (!lastLoadedFile.isEmpty())
    {
        // qDebug("writing last loaded file");
        settings.setValue("loadfile", lastLoadedFile);
    }
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::save(Json::Value& json)
{
    QTabWidget* tabmanager = (QTabWidget*)getNamedWidget("tabmanager");
    for (int i = 0; i < tabmanager->count(); ++i)
    {
        ModeContainer* mode = (ModeContainer*)tabmanager->widget(i);
        Json::Value modejson;
        mode->save(modejson);
        json["modes"].append(modejson);
    }
    json["currentMode"] = tabmanager->currentIndex();
}

void MainWindow::restore(Json::Value& json)
{
    /* clear current task view */
    QTabWidget* tabmanager = (QTabWidget*)getNamedWidget("tabmanager");
    for (int i = 0; i < tabmanager->count(); ++i)
    {
        delete tabmanager->widget(i);
    }
    tabmanager->clear();

    /* repopulate task view */
    if (json.isMember("modes"))
    {
        for (int i = 0; i < json["modes"].size(); ++i)
        {
            Json::Value modejson = json["modes"][i];
            tabmanager->addTab(
                new ModeContainer(modejson, this),
                QString::fromStdString(modejson.get("name", "").asString()));
        }
    }
    tabmanager->setCurrentIndex(json.get("currentMode", 0).asInt());

    /* show tutorial page if no modes are restored */
    if (ui.tabmanager->count() == 0)
    {
        QString name = "Tutorial";
        ModeContainer* tutorialMode = new ModeContainer(name);
        ui.tabmanager->addTab(tutorialMode, name);
    }
}

void MainWindow::restore(QString filename)
{
    Json::CharReaderBuilder rb;
    Json::Value json;
    JSONCPP_STRING errs;

    std::ifstream file;
    file.open(filename.toStdString());
    if (file.is_open() && Json::parseFromStream(rb, file, &json, &errs))
    {
        restore(json);
    }
}

}    // namespace insitu
