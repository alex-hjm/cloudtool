#include "mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>
#include <vld.h>

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
