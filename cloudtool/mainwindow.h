#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTranslator>

#include "base/customdock.h"
#include "base/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void changeTheme(int);
    void changeLanguage(int);
    void saveScreenshot();

    template <class T>
    void createLeftDock(const QString& label)
    {
        ct::createDock<T>(this, label, ui->cloudview, ui->cloudtree, ui->console,
                          Qt::LeftDockWidgetArea, ui->PropertiesDock);
    }

    template <class T>
    void createRightDock(const QString& label)
    {
        ct::createDock<T>(this, label, ui->cloudview, ui->cloudtree, ui->console,
                          Qt::RightDockWidgetArea);
    }

    template <class T>
    void createDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console);
    }

protected:
    void moveEvent(QMoveEvent* event);

private:
    Ui::MainWindow* ui;
    QTranslator* translator;
};
#endif // MAINWINDOW_H
