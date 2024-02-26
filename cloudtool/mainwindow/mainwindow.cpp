/**
 * @file mainwindow.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-26
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>

#define DEFAULT_WIN_WIDTH   1056
#define DEFAULT_WIN_HEIGHT  720

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), 
    ui(new Ui::MainWindow),
    m_setting(new Setting(this))
{
    ui->setupUi(this);

    resize(DEFAULT_WIN_WIDTH, DEFAULT_WIN_HEIGHT);
    setMinimumSize(DEFAULT_WIN_WIDTH / 2, DEFAULT_WIN_HEIGHT / 2);

    QList<QDockWidget*> docks {ui->DataDock, ui->PropertiesDock, ui->ConsoleDock};
    QList<int> size {DEFAULT_WIN_HEIGHT * 2 / 5, DEFAULT_WIN_HEIGHT * 2 / 5, DEFAULT_WIN_HEIGHT * 1 / 5};
    resizeDocks(docks, size, Qt::Orientation::Vertical);

    ui->DataDock->setMinimumWidth(DEFAULT_WIN_WIDTH / 4);
    ui->PropertiesDock->setMinimumWidth(DEFAULT_WIN_WIDTH / 4);

    // add setting action
    QWidget *midSpacerWidget(new QWidget());
    midSpacerWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->toolBar->addWidget(midSpacerWidget);
    ui->toolBar->addAction(ui->actionSetting);
    QWidget *lastSpacerWidget(new QWidget());
    lastSpacerWidget->setMinimumHeight(9);
    ui->toolBar->addWidget(lastSpacerWidget);

    // connect ui signals to slots
    connect(ui->cloudlist, &ct::CloudList::addCloudEvent, ui->cloudview, &ct::CloudView::addCloud);
    connect(ui->cloudlist, &ct::CloudList::removeCloudEvent, ui->cloudview, &ct::CloudView::removeCloud);
    connect(ui->cloudlist, &ct::CloudList::addCloudBBoxEvent, ui->cloudview, &ct::CloudView::addCloudBBox);
    connect(ui->cloudlist, &ct::CloudList::removeCloudBBoxEvent, ui->cloudview, &ct::CloudView::removeCloudBBox);
    connect(ui->cloudlist, &ct::CloudList::selectCloudEvent, ui->cloudtable, &ct::CloudTable::handleSelectCloud);
    connect(ui->cloudtable, &ct::CloudTable::updateCloudEvent, ui->cloudview, &ct::CloudView::addCloud);
    connect(ui->cloudlist, &ct::CloudList::logging, ui->console, &ct::Console::logging);
    connect(ui->cloudtable, &ct::CloudTable::logging, ui->console, &ct::Console::logging);
    connect(m_setting, &Setting::logging, ui->console, &ct::Console::logging);
    connect(m_setting, &Setting::changeLanguageEvent, this, [=] { ui->retranslateUi(this); } );

    // connect action signals to slots
    connect(ui->actionOpen, &QAction::triggered, ui->cloudlist, &ct::CloudList::loadCloud);
    connect(ui->actionClose, &QAction::triggered, ui->cloudlist, &ct::CloudList::removeSelectedClouds);
    connect(ui->actionClear, &QAction::triggered, ui->cloudlist, &ct::CloudList::removeAllClouds);
    connect(ui->actionSave, &QAction::triggered, ui->cloudlist, &ct::CloudList::saveSelectedClouds);
    connect(ui->actionMerge, &QAction::triggered, ui->cloudlist, &ct::CloudList::mergeSelectedClouds);
    connect(ui->actionClone, &QAction::triggered, ui->cloudlist, &ct::CloudList::cloneSelectedClouds);


    connect(ui->actionSetting, &QAction::triggered, m_setting, &Setting::show);
    m_setting->loadSetting();
    ui->console->logging(ct::LOG_INFO, tr("load cloud success!"));
}

MainWindow::~MainWindow()
{
    delete ui;
}
