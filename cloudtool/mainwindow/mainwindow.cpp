#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDesktopWidget>
#include <QFile>

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), 
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QDesktopWidget *desktop = QApplication::desktop();
    QRect mainScreenSize = desktop->screenGeometry(desktop->primaryScreen());
    int mainWidth = mainScreenSize.width() * 1 / 2;
    int mainHeight = mainScreenSize.height() * 3 / 5;
    resize(mainWidth, mainHeight);
    setMinimumSize(mainWidth / 2, mainHeight / 2);

    QList<QDockWidget*> docks {ui->DataDock, ui->PropertiesDock, ui->ConsoleDock};
    QList<int> size {mainHeight * 2 / 5, mainHeight * 2 / 5, mainHeight * 1 / 5};
    resizeDocks(docks, size, Qt::Orientation::Vertical);

    ui->DataDock->setMinimumWidth(mainWidth / 4);
    ui->PropertiesDock->setMinimumWidth(mainWidth / 4);

    QObject::connect(ui->cloudlist, &ct::CloudList::addCloudEvent, ui->cloudview, &ct::CloudView::addCloud);
    QObject::connect(ui->cloudlist, &ct::CloudList::removeCloudEvent, ui->cloudview, &ct::CloudView::removeCloud);
    QObject::connect(ui->cloudlist, &ct::CloudList::addCloudBBoxEvent, ui->cloudview, &ct::CloudView::addCloudBBox);
    QObject::connect(ui->cloudlist, &ct::CloudList::removeCloudBBoxEvent, ui->cloudview, &ct::CloudView::removeCloudBBox);
    QObject::connect(ui->cloudlist, &ct::CloudList::selectCloudEvent, ui->cloudtable, &ct::CloudTable::handleSelectCloud);
    QObject::connect(ui->cloudtable, &ct::CloudTable::updateCloudEvent, ui->cloudview, &ct::CloudView::addCloud);
    QObject::connect(ui->cloudlist, &ct::CloudList::logging, ui->console, &ct::Console::logging);

    QFile qss;
    qss.setFileName(":/res/theme/light.qss");
    qss.open(QFile::ReadOnly);
    qApp->setStyleSheet(qss.readAll());
    qss.close();
}

MainWindow::~MainWindow()
{
    delete ui;
}
