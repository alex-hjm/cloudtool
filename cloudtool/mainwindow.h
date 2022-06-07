#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTranslator>

#include "base/customdock.h"
#include "base/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  void changeTheme(int);
  void changeLanguage(int);

  template <class T>
  void createLeftDock(const QString& label)
  {
    ct::createDock<T>(this, label, Qt::LeftDockWidgetArea, ui->console,
                       ui->cloudview, ui->cloudtree, ui->PropertiesDock);
  }

  template <class T>
  void createRightDock(const QString& label)
  {
    ct::createDock<T>(this, label, Qt::RightDockWidgetArea,
                       ui->cloudview, ui->cloudtree, ui->console);
  }

  template <class T>
  void createDialog(const QString& label)
  {
    typedef void (MainWindow::* SignalType)(const QPoint&);
    SignalType signal = &MainWindow::posChanged;
    ct::createDialog<T, SignalType>(this, signal, label, ui->centralWidget->pos(),
                                     ui->cloudview, ui->cloudtree, ui->console);
  }

signals:
  void posChanged(const QPoint&);

protected:
  void moveEvent(QMoveEvent* event);

private:
  Ui::MainWindow* ui;
  QTranslator* translator;
};
#endif  // MAINWINDOW_H
