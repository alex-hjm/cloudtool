/**
 * @file mainwindow.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-26
 */
#ifndef _CT_MAINWINDOW_H_
#define _CT_MAINWINDOW_H_

#include <QMainWindow>
#include "setting/setting.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    Setting* m_setting;
};
#endif // _CT_MAINWINDOW_H_
