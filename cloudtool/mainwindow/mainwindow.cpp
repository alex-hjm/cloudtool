#include "mainwindow.h"
#include "ui_mainwindow.h"

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QScreen>
#else
#include <QDesktopWidget>
#endif
#include <QFile>

#define DEFAULT_WIN_WIDTH   1056
#define DEFAULT_WIN_HEIGHT  720

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), 
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
#else
    QDesktopWidget *desktop = QApplication::desktop();
    QRect screenGeometry = desktop->screenGeometry();
#endif

    resize(DEFAULT_WIN_WIDTH, DEFAULT_WIN_HEIGHT);
    setMinimumSize(DEFAULT_WIN_WIDTH / 2, DEFAULT_WIN_HEIGHT / 2);

    QList<QDockWidget*> docks {ui->DataDock, ui->PropertiesDock, ui->ConsoleDock};
    QList<int> size {DEFAULT_WIN_HEIGHT * 2 / 5, DEFAULT_WIN_HEIGHT * 2 / 5, DEFAULT_WIN_HEIGHT * 1 / 5};
    resizeDocks(docks, size, Qt::Orientation::Vertical);

    ui->DataDock->setMinimumWidth(DEFAULT_WIN_WIDTH / 4);
    ui->PropertiesDock->setMinimumWidth(DEFAULT_WIN_WIDTH / 4);

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
