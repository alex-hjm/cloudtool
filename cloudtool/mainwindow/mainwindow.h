/**
 * @file mainwindow.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-26
 */
#ifndef _CT_MAINWINDOW_H_
#define _CT_MAINWINDOW_H_

#include <QMainWindow>
#include <QMutex>
#include <QListWidget>
#include <map>

#include "setting/setting.h"
#include "common/customwidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void open();
    void close();
    void save();
    void clear();
    void rename();
    void merge();
    void clone();

    void saveScreenshot();
    void saveCameraParam();
    void loadCameraParam();

public slots:
    void handleThemeChanged(Setting::Theme theme);
    void handleCloudSelectionChanged();
    void handleCloudChanged(QListWidgetItem *);
    void showCloudListMenu(const QPoint &pos);
    void showCloudViewMenu(const QPoint &pos);
    void showConsoleMenu(const QPoint &pos);

private:
    template <class T>
    void createLeftDock(const QString& label);

    template <class T>
    void createRightDock(const QString& label);

    template <class T>
    void createDialog(const QString& label);

signals:
    void posChanged(const QPoint &pos);

protected:
    void moveEvent(QMoveEvent* event);

private:
    Ui::MainWindow *ui;
    Setting* m_setting;
    QMutex m_mutex;
    std::map<QString, CustomDock*> m_left_docks;
    std::map<QString, CustomDock*> m_right_docks;
    std::vector<CustomDialog*> m_dialogs;
};
#endif // _CT_MAINWINDOW_H_
