/**
 * @file mainwindow.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-26
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "module/fileio.h"
#include "common/resource.h"

#include <QFile>
#include <QFileDialog>
#include <QThread>
#include <QMessageBox>
#include <QPushButton>
#include <QInputDialog>
#include <QLabel>
#include <QSpinBox>
#include <QMenu>

#define DEFAULT_WIN_WIDTH   1056
#define DEFAULT_WIN_HEIGHT  720

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), 
    ui(new Ui::MainWindow),
    m_setting(new Setting(this))
{
    ui->setupUi(this);

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    // resize window
    resize(DEFAULT_WIN_WIDTH, DEFAULT_WIN_HEIGHT);
    setMinimumSize(DEFAULT_WIN_WIDTH / 2, DEFAULT_WIN_HEIGHT / 2);

    QList<QDockWidget*> docks {ui->DataDock, ui->PropertiesDock, ui->ConsoleDock};
    QList<int> size {DEFAULT_WIN_HEIGHT * 2 / 5, DEFAULT_WIN_HEIGHT * 2 / 5, DEFAULT_WIN_HEIGHT * 1 / 5};
    resizeDocks(docks, size, Qt::Orientation::Vertical);

    ui->DataDock->setMinimumWidth(DEFAULT_WIN_WIDTH / 4);
    ui->PropertiesDock->setMinimumWidth(DEFAULT_WIN_WIDTH / 4);

    // cloudtable
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeMode::Stretch);
    ui->cloudtable->horizontalHeader()->setStretchLastSection(true);
    ui->cloudtable->verticalHeader()->setHidden(true);
    ui->cloudtable->setShowGrid(false);
    ui->cloudtable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->cloudtable->setSelectionMode(QAbstractItemView::NoSelection);

    // cloudlist
    ui->cloudlist->setCloudIcon(QIcon(ICON_CLOUD));
    connect(ui->cloudlist, &ct::CloudList::itemSelectionChanged, this, &MainWindow::handleCloudSelectionChanged);
    connect(ui->cloudlist, &ct::CloudList::itemChanged, this, &MainWindow::handleCloudChanged);
    connect(ui->cloudlist, &ct::CloudList::customContextMenuRequested, this, &MainWindow::showCloudListMenu);

    // action
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::open);
    connect(ui->actionClose, &QAction::triggered, this, &MainWindow::close);
    connect(ui->actionClear, &QAction::triggered, this, &MainWindow::clear);
    connect(ui->actionSave, &QAction::triggered, this, &MainWindow::save);
    connect(ui->actionMerge, &QAction::triggered, this, &MainWindow::merge);
    connect(ui->actionClone, &QAction::triggered, this, &MainWindow::clone);
    connect(ui->actionRename, &QAction::triggered, this, &MainWindow::rename);

    // setting
    QWidget *midSpacerWidget(new QWidget());
    midSpacerWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->toolBar->addWidget(midSpacerWidget);
    ui->toolBar->addAction(ui->actionSetting);
    QWidget *lastSpacerWidget(new QWidget());
    lastSpacerWidget->setMinimumHeight(9);
    ui->toolBar->addWidget(lastSpacerWidget);
    connect(m_setting, &Setting::changeLanguageEvent, this, [=] { ui->retranslateUi(this); });
    connect(m_setting, &Setting::logging, ui->console, &ct::Console::logging);
    connect(ui->actionSetting, &QAction::triggered, m_setting, &Setting::show);
    m_setting->loadSetting();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::open()
{
    QString filter = "all(*.*);;ply(*.ply);;pcd(*.pcd)";
    QString lastOpenDir = m_setting->value("lastOpenDir", DEFAULT_DATA_PATH).toString();
    QStringList fileList = QFileDialog::getOpenFileNames(this, tr("Open file"), lastOpenDir, filter);
    for(auto& file : fileList) {
        QThread *thread = new QThread;
        ct::FileIO *fileio = new ct::FileIO;
        fileio->moveToThread(thread);
        connect(thread, &QThread::finished, fileio, &QObject::deleteLater);
        connect(thread, &QThread::started, [=] { fileio->loadCloudFile(file); });
        connect(fileio, &ct::FileIO::loadCloudResult, this, [=](bool res, const ct::Cloud::Ptr& cloud) {
            QMutexLocker locker(&m_mutex);
            if (!res || !ui->cloudlist->appendCloud(cloud)) {
                ui->console->logging(ct::LOG_ERROR, tr("Failed to load cloud file: %1").arg(file));
            } else {
                ui->cloudview->addCloud(cloud);
                ui->cloudview->addCloudBBox(cloud);
                ui->console->logging(ct::LOG_INFO, tr("Success to load cloud file: %1").arg(file));
            }
            thread->quit();
            thread->wait();
        }, Qt::QueuedConnection);
        thread->start();
        m_setting->setValue("lastOpenDir", QFileInfo(file).path());
    }
}

void MainWindow::close()
{
    auto clouds = ui->cloudlist->getSelectedClouds();
    for(auto& cloud : clouds) {
        ui->cloudlist->removeCloud(cloud);
        ui->cloudview->removeCloud(cloud->id());
        ui->cloudview->removeCloudBBox(cloud->bboxId());
        ui->console->logging(ct::LOG_INFO, tr("Success to remove cloud: %1").arg(cloud->id()));
    }
}

void MainWindow::save()
{
    auto clouds = ui->cloudlist->getSelectedClouds();
    for(auto& cloud : clouds) {
        QString filter = "ply(*.ply);;pcd(*.pcd)";
        QString lastOpenDir = m_setting->value("lastOpenDir", DEFAULT_DATA_PATH).toString();
        QString id = cloud->id(), bboxId = cloud->bboxId();
        QString file = QFileDialog::getSaveFileName(this, tr("Save file"), lastOpenDir + "/" + id, filter);
        if (file.isEmpty()) continue;
        QMessageBox messageBox(QMessageBox::NoIcon, PROJECT_NAME, tr("Save in binary or ascii format"),
                                QMessageBox::NoButton, this);
        messageBox.addButton(tr("Ascii"), QMessageBox::ActionRole);
        messageBox.addButton(tr("Binary"), QMessageBox::ActionRole)->setDefault(true);
        messageBox.addButton(tr("Close"), QMessageBox::RejectRole)->setHidden(true);
        int isBinary = messageBox.exec();
        if (2 == isBinary) continue;
        QThread *thread = new QThread;
        ct::FileIO *fileio = new ct::FileIO;
        fileio->moveToThread(thread);
        connect(thread, &QThread::finished, fileio, &QObject::deleteLater);
        connect(thread, &QThread::started, [=] { fileio->saveCloudFile(cloud, file, isBinary);});
        connect(fileio, &ct::FileIO::saveCloudResult, this, [=](bool res) {
            QMutexLocker locker(&m_mutex);
            if (!res || !ui->cloudlist->renameCloud(cloud, QFileInfo(file).baseName())) {
                ui->console->logging(ct::LOG_ERROR, tr("Failed to save cloud file: %1").arg(file));
            } else {
                ui->cloudview->removeCloud(id);
                ui->cloudview->removeCloudBBox(bboxId);
                ui->cloudview->addCloud(cloud);
                ui->cloudview->addCloudBBox(cloud);
                ui->console->logging(ct::LOG_INFO, tr("Success to save cloud file: %1").arg(file));
            }
            thread->quit();
            thread->wait();
        }, Qt::QueuedConnection);
        thread->start();
        m_setting->setValue("lastOpenDir", QFileInfo(file).path());
    }
}

void MainWindow::clear()
{
    auto clouds = ui->cloudlist->getAllClouds();
    for(auto& cloud : clouds) {
        ui->cloudlist->removeCloud(cloud);
        ui->cloudview->removeCloud(cloud->id());
        ui->cloudview->removeCloudBBox(cloud->bboxId());
    }
    ui->console->logging(ct::LOG_INFO, tr("Success to clear clouds"));
}

void MainWindow::rename()
{
    auto clouds = ui->cloudlist->getAllClouds();
    for(auto& cloud : clouds) {
        bool ok = false;
        QString id = cloud->id(), bboxId = cloud->bboxId();
        QString rename = QInputDialog::getText(this, PROJECT_NAME, tr("Rename: "), QLineEdit::Normal, cloud->id(), &ok);
        if (!ok)  continue;
        if (ui->cloudlist->renameCloud(cloud, rename)) {
            ui->cloudview->removeCloud(id);
            ui->cloudview->removeCloudBBox(bboxId);
            ui->cloudview->addCloud(cloud);
            ui->cloudview->addCloudBBox(cloud);
            ui->console->logging(ct::LOG_INFO, tr("Success to rename cloud: %1 -> %2").arg(id).arg(cloud->id()));
        } else {
            ui->console->logging(ct::LOG_ERROR, tr("Failed to rename cloud: %1 -> %2").arg(id).arg(rename));
        }
    }
}

void MainWindow::merge()
{
    auto clouds = ui->cloudlist->getSelectedClouds();
    auto merged_cloud = ui->cloudlist->mergeClouds(clouds);
    if (merged_cloud == nullptr) {
        ui->console->logging(ct::LOG_ERROR, tr("Failed to merge clouds"));
    } else {
        ui->cloudview->addCloud(merged_cloud);
        ui->cloudview->addCloudBBox(merged_cloud);
        ui->console->logging(ct::LOG_INFO, tr("Success to merge clouds: %1").arg(merged_cloud->id()));
    }
}

void MainWindow::clone()
{
    auto clouds = ui->cloudlist->getSelectedClouds();
    for(auto& cloud : clouds) {
        auto cloned_cloud = ui->cloudlist->cloneCloud(cloud);
        if (cloned_cloud == nullptr) {
            ui->console->logging(ct::LOG_ERROR, tr("Failed to clone cloud: %1").arg(cloud->id()));
        } else {
            ui->cloudview->addCloud(cloned_cloud);
            ui->cloudview->addCloudBBox(cloned_cloud);
            ui->console->logging(ct::LOG_INFO, tr("Success to clone cloud: %1").arg(cloned_cloud->id()));
        }
    }
}

void MainWindow::handleCloudSelectionChanged()
{
    for(int i = 0; i < ui->cloudlist->count(); i++) { 
        if (ui->cloudlist->item(i)->isSelected()) {
            ui->cloudview->addCloudBBox(ui->cloudlist->getCloud(i));
        } else {
            ui->cloudview->removeCloudBBox(ui->cloudlist->getCloud(i)->bboxId());
        }
    }

    for (int i = 0; i < ui->cloudtable->rowCount(); i++) {
        ui->cloudtable->removeCellWidget(i, 1);
    }

    auto clouds = ui->cloudlist->getSelectedClouds();
    if (clouds.size() == 0) return;
    // id
    QLabel* id = new QLabel(clouds.back()->id(), this);
    id->setAlignment(Qt::AlignCenter);
    ui->cloudtable->setCellWidget(0, 1, id);
    // pointNum
    QLabel* pointNum = new QLabel(QString::number(clouds.back()->pointNum()), this);
    pointNum->setAlignment(Qt::AlignCenter);
    ui->cloudtable->setCellWidget(1, 1, pointNum);
    // resolution
    QLabel* resolution = new QLabel(QString::number(clouds.back()->resolution(), 'f', 8), this);
    resolution->setAlignment(Qt::AlignCenter);
    ui->cloudtable->setCellWidget(2, 1, resolution);
    // pointSize
    QSpinBox* pointSize = new QSpinBox(this);
    pointSize->setValue(clouds.back()->pointSize());
    pointSize->setRange(1, 99);
    pointSize->setFrame(false);
    pointSize->setButtonSymbols(QAbstractSpinBox::NoButtons);
    pointSize->setAlignment(Qt::AlignCenter);
    connect(pointSize, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=](int value) {
                clouds.back()->setPointSize(value);
                ui->cloudview->addCloud(clouds.back());
            });
    ui->cloudtable->setCellWidget(3, 1, pointSize);
    // opacity
    QDoubleSpinBox* opacity = new QDoubleSpinBox(this);
    opacity->setValue(clouds.back()->opacity());
    opacity->setRange(0, 1);
    opacity->setSingleStep(0.1);
    opacity->setDecimals(1);
    opacity->setFrame(false);
    opacity->setButtonSymbols(QAbstractSpinBox::NoButtons);
    opacity->setAlignment(Qt::AlignCenter);
    connect(opacity, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value) {
                clouds.back()->setOpacity(value);
                ui->cloudview->addCloud(clouds.back());
            });
    ui->cloudtable->setCellWidget(4, 1, opacity);
}

void MainWindow::handleCloudChanged(QListWidgetItem *item)
{
    auto cloud = ui->cloudlist->getCloud(ui->cloudlist->row(item));
    if (cloud == nullptr) return;
    if (item->checkState() == Qt::Checked) {
        ui->cloudview->addCloud(cloud);
    } else if (item->checkState() == Qt::Unchecked) {
        ui->cloudview->removeCloud(cloud->id());
    }
    if (item->text() != cloud->id()) {
        ui->cloudview->removeCloud(cloud->id());
        ui->cloudview->removeCloudBBox(cloud->bboxId());
        cloud->setId(item->text());
        ui->cloudview->addCloud(cloud);
        ui->cloudview->addCloudBBox(cloud);
    }
    handleCloudSelectionChanged();
}

void MainWindow::showCloudListMenu(const QPoint &pos)
{
    QMenu* menu = new QMenu(this);
    menu->addAction(QIcon(ICON_OPEN), tr("Open"), this, &MainWindow::open);
    menu->addAction(QIcon(ICON_CLOSE), tr("Close"), this, &MainWindow::close);
    menu->addAction(QIcon(ICON_CLEAR), tr("Clear"), this, &MainWindow::clear);
    menu->addAction(QIcon(ICON_SAVE), tr("Save"), this, &MainWindow::save);
    menu->addAction(QIcon(ICON_RENAME), tr("Rename"), this, &MainWindow::rename);
    menu->addAction(QIcon(ICON_MERGE), tr("Merge"), this, &MainWindow::merge);
    menu->addAction(QIcon(ICON_CLONE), tr("Clone"), this, &MainWindow::clone);
    menu->exec(ui->cloudlist->mapFromGlobal(pos));
}

