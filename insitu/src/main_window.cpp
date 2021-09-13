
#include <json/reader.h>
#include <qfiledialog.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "insitu_utils.hpp"
#include "main_window.hpp"
#include "add_mode_dialog.hpp"

namespace insitu
{
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUI(this);

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tabmanager->setCurrentIndex(0);
}

MainWindow::~MainWindow()
{
}

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
    QTabWidget* tabmanager = (QTabWidget*)getNamedWidget("tabmanager");
    for (int i = 0; i < tabmanager->count(); ++i) {
        ModeContainer* mode = (ModeContainer*)tabmanager->widget(i);
        Json::Value modejson;
        mode->save(modejson);
        json["modes"].append(modejson);
    }
    json["currentMode"] = tabmanager->currentIndex();

    std::ofstream file;
    file.open(QFileDialog::getSaveFileName(this, tr("Save File"), QDir::currentPath(), tr("JSON (*.json)")).toStdString());
    if (file.is_open()) {
        Json::StreamWriterBuilder sb;
        sb["indentation"] = "    ";
        auto writer = sb.newStreamWriter();
        writer->write(json, &file);
        file.close();
    }
}

void MainWindow::on_actionLoad_triggered()
{
    Json::CharReaderBuilder rb;
    Json::Value json;
    JSONCPP_STRING errs;

    std::ifstream file;
    file.open(QFileDialog::getOpenFileName(this, tr("Open File"), QDir::currentPath(), tr("JSON (*.json)")).toStdString());
    if (file.is_open() && Json::parseFromStream(rb, file, &json, &errs)) {
        /* clear current task view */
        QTabWidget* tabmanager = (QTabWidget*)getNamedWidget("tabmanager");
        for (int i = 0; i < tabmanager->count(); ++i) {
            delete tabmanager->widget(i);
        }
        tabmanager->clear();

        /* repopulate task view */
        if (json.isMember("modes")) {
            for (int i = 0; i < json["modes"].size(); ++i) {
                Json::Value modejson = json["modes"][i];
                tabmanager->addTab(new ModeContainer(modejson, this), QString::fromStdString(modejson.get("name", "").asString()));
            }
        }
        tabmanager->setCurrentIndex(json.get("currentMode", 0).asInt());
    }
}

void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "insitu");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "insitu");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}    // namespace insitu

