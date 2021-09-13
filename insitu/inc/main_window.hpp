
#ifndef insitu_MAIN_WINDOW_H
#define insitu_MAIN_WINDOW_H

#include <QMainWindow>
#include <QInputDialog>
#include "main_window_design.hpp"

/* C++ includes */
#include <json/json.h>

namespace insitu
{
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget* parent = 0);
    ~MainWindow();

    void ReadSettings(void);
    
    void WriteSettings(void);

    void closeEvent(QCloseEvent* event);

public Q_SLOTS:
    void on_actionNewMode_triggered(void);
    
    void on_actionNewView_triggered(void);
    
    void on_actionNewFilter_triggered(void);
    
    void on_actionAbout_triggered(void);
    
    void on_actionSave_triggered(void);
    
    void on_actionLoad_triggered(void);

private:
    ui::main_window_design ui;

    QString lastLoadedFile;

    QStringList recentFiles;

    void save(Json::Value& json);

    void restore(Json::Value& json);

    void restore(QString filename);

};

}    // namespace insitu

#endif    // insitu_MAIN_WINDOW_H

