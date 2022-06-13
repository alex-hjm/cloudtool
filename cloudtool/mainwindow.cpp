#include "mainwindow.h"

#include "edit/boundingbox.h"
#include "edit/color.h"
#include "edit/coordinate.h"
#include "edit/normals.h"
#include "edit/scale.h"
#include "edit/transformation.h"

#include "help/about.h"
#include "help/shortcutkey.h"

#include "tool/filters.h"
//#include "tools/keypoints.h"

#include <QDebug>
#include <QDesktopWidget>
#include <QDir>
#include <QFile>

#include "ui_mainwindow.h"

#define PARENT_ICON_PATH    ":/res/icon/document-open.svg"
#define CHILD_ICON_PATH     ":/res/icon/view-calendar.svg"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      translator(nullptr)
{
    ui->setupUi(this);

    // resize
    this->setBaseSize(1320, 845);
    QList<QDockWidget *> docks;
    docks.push_back(ui->DataDock);
    docks.push_back(ui->PropertiesDock);
    docks.push_back(ui->ConsoleDock);
    QList<int> size;
    size.push_back(300);
    size.push_back(320);
    size.push_back(140);
    resizeDocks(docks, size, Qt::Orientation::Vertical);

    // init qrc
    Q_INIT_RESOURCE(res);

    // connect pointer
    ui->cloudtree->setCloudView(ui->cloudview);
    ui->cloudtree->setConsole(ui->console);
    ui->cloudtree->setPropertiesTable(ui->cloudtable);
    ui->cloudtree->setProgressBar(ui->progress_bar);
    ui->cloudtree->setParentIcon(QIcon(PARENT_ICON_PATH));
    ui->cloudtree->setChildIcon(QIcon(CHILD_ICON_PATH));

    // toolbar
    // file
    connect(ui->actionOpen, &QAction::triggered, ui->cloudtree, &ct::CloudTree::addCloud);
    connect(ui->actionSample, &QAction::triggered, ui->cloudtree, &ct::CloudTree::addSampleCloud);
    connect(ui->actionSave, &QAction::triggered, ui->cloudtree, &ct::CloudTree::saveSelectedClouds);
    connect(ui->actionClose, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeSelectedClouds);
    connect(ui->actionCloseAll, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeAllClouds);
    connect(ui->actionMerge, &QAction::triggered, ui->cloudtree, &ct::CloudTree::mergeSelectedClouds);
    connect(ui->actionClone, &QAction::triggered, ui->cloudtree, &ct::CloudTree::cloneSelectedClouds);
    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);

    // edit
    connect(ui->actionBoundingBox, &QAction::triggered, [=] { this->createLeftDock<BoundingBox>("BoundingBox"); });
    connect(ui->actionColor, &QAction::triggered, [=] { this->createLeftDock<Color>("Color"); });
    connect(ui->actionCoordinate, &QAction::triggered, [=] { this->createDialog<Coordinate>("Coordinate"); });
    connect(ui->actionNormals, &QAction::triggered, [=] { this->createLeftDock<Normals>("Normals"); });
    connect(ui->actionScale, &QAction::triggered, [=] { this->createDialog<Scale>("Scale"); });
    connect(ui->actionTransformation, &QAction::triggered, [=] { this->createLeftDock<Transformation>("Transformation"); });

    // view
    connect(ui->actionResetcamera, &QAction::triggered, ui->cloudview, &ct::CloudView::resetCamera);
    connect(ui->actionTopView, &QAction::triggered, ui->cloudview, &ct::CloudView::setTopView);
    connect(ui->actionFrontView, &QAction::triggered, ui->cloudview, &ct::CloudView::setFrontView);
    connect(ui->actionLeftSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setLeftSideView);
    connect(ui->actionBackView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBackView);
    connect(ui->actionRightSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setRightSideView);
    connect(ui->actionBottomView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBottomView);
    connect(ui->actionShowAxes, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowAxes);
    connect(ui->actionShowFPS, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowFPS);
    connect(ui->actionShowId, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowId);

    // tools
    connect(ui->actionFilters, &QAction::triggered, [=] { this->createLeftDock<Filters>("Filters"); });

    // options
    connect(ui->actionOrigin, &QAction::triggered, [=] { changeTheme(0); });
    connect(ui->actionLight, &QAction::triggered, [=] { changeTheme(1); });
    connect(ui->actionDark, &QAction::triggered, [=] { changeTheme(2); });
    connect(ui->actionEnglish, &QAction::triggered, [=] { changeLanguage(0); });
    connect(ui->actionChinese, &QAction::triggered, [=] { changeLanguage(1); });

    // help
    About *about = new About(this);
    ShortcutKey *shortcutKey = new ShortcutKey(this);
    connect(ui->actionAbout, &QAction::triggered, about, &QDialog::show);
    connect(ui->actionShortcutKey, &QAction::triggered, shortcutKey, &QDialog::show);

    this->changeTheme(1);
    ui->progress_bar->close();
    ui->console->print(ct::LOG_INFO, "CloudTool started!");
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::changeTheme(int index)
{
    QFile qss;
    switch (index)
    {
    case 0:
        qss.setFileName(":/res/theme/origin.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("Origin Theme"), 2000);
        break;
    case 1:
        qss.setFileName(":/res/theme/light.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("Light Theme"), 2000);
        break;
    case 2:
        qss.setFileName(":/res/theme/dark.qss");
        qss.open(QFile::ReadOnly);
        qApp->setStyleSheet(qss.readAll());
        qss.close();
        ui->statusBar->showMessage(tr("Dark Theme"), 2000);
        break;
    }
}

void MainWindow::changeLanguage(int index)
{
    switch (index)
    {
    case 0:
        if (translator != nullptr)
        {
            qApp->removeTranslator(translator);
            ui->retranslateUi(this);
        }
        break;
    case 1:
        if (translator == nullptr)
        {
            translator = new QTranslator;
            translator->load(":/res/trans/zh_CN.qm");
        }
        qApp->installTranslator(translator);
        ui->retranslateUi(this);
        break;
    }
}

void MainWindow::moveEvent(QMoveEvent *event)
{
    QPoint pos = mapToParent(ui->centralWidget->pos());
    emit posChanged(pos);
    return QMainWindow::moveEvent(event);
}
