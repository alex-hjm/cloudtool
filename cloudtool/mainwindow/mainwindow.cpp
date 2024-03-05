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
#include <QShortcut>
#include <QDateTime>
#include <QMoveEvent>

#include "edit/color.h"

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

    // cloudview
    connect(ui->cloudview, &ct::CloudView::customContextMenuRequested, this, &MainWindow::showCloudViewMenu);

    // console
    connect(ui->console, &ct::Console::customContextMenuRequested, this, &MainWindow::showConsoleMenu);

    // setting
    QWidget *midSpacerWidget(new QWidget());
    midSpacerWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->toolBar->addWidget(midSpacerWidget);
    ui->toolBar->addAction(ui->actionSetting);
    QWidget *lastSpacerWidget(new QWidget());
    lastSpacerWidget->setMinimumHeight(9);
    ui->toolBar->addWidget(lastSpacerWidget);
    connect(m_setting, &Setting::changeLanguageEvent, this, [=] { ui->retranslateUi(this); });
    connect(m_setting, &Setting::changeThemeEvent, this, &MainWindow::handleThemeChanged);
    connect(m_setting, &Setting::logging, ui->console, &ct::Console::logging);
    connect(ui->actionSetting, &QAction::triggered, m_setting, &Setting::show);
    m_setting->loadSetting();

    //shortcut
    connect(new QShortcut(QKeySequence::Open, this), &QShortcut::activated, this, &MainWindow::open);
    connect(new QShortcut(QKeySequence::Delete, this), &QShortcut::activated, this, &MainWindow::close);
    connect(new QShortcut(QKeySequence::Save, this), &QShortcut::activated, this, &MainWindow::save);
    connect(new QShortcut(Qt::CTRL + Qt::Key_R, this), &QShortcut::activated, this, &MainWindow::rename);
    connect(new QShortcut(Qt::CTRL + Qt::Key_M, this), &QShortcut::activated, this, &MainWindow::merge);
    connect(new QShortcut(Qt::CTRL + Qt::Key_C, this), &QShortcut::activated, this, &MainWindow::clone);

    // action
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::open);
    connect(ui->actionClose, &QAction::triggered, this, &MainWindow::close);
    connect(ui->actionClear, &QAction::triggered, this, &MainWindow::clear);
    connect(ui->actionSave, &QAction::triggered, this, &MainWindow::save);
    connect(ui->actionRename, &QAction::triggered, this, &MainWindow::rename);
    connect(ui->actionMerge, &QAction::triggered, this, &MainWindow::merge);
    connect(ui->actionClone, &QAction::triggered, this, &MainWindow::clone);
    connect(ui->actionColor, &QAction::triggered, this, [=]{ createLeftDock<Color>("Color"); });

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

void MainWindow::saveScreenshot()
{
    QString lastOpenDir = m_setting->value("lastOpenDir", DEFAULT_DATA_PATH).toString();
    QString fileName = lastOpenDir + "/screenshot" + QDateTime::currentDateTime().toString("-hh-mm-ss");
    QString file = QFileDialog::getSaveFileName(this, tr("Save Screenshot"), fileName, "PNG(*.png);;JPG(*.jpg)");
    if (file.isEmpty()) return;
    ui->cloudview->saveScreenshot(file.toLocal8Bit());
    m_setting->setValue("lastOpenDir", QFileInfo(file).path());
}

void MainWindow::saveCameraParam()
{
    QString lastOpenDir = m_setting->value("lastOpenDir", DEFAULT_DATA_PATH).toString();
    QString fileName = lastOpenDir + "/camparam" + QDateTime::currentDateTime().toString("-hh-mm-ss");
    QString file = QFileDialog::getSaveFileName(this, tr("Save Camera Parameter"), fileName, "CAM(*.cam)");
    if (file.isEmpty()) return;
    ui->cloudview->saveCameraParam(file.toLocal8Bit());
    m_setting->setValue("lastOpenDir", QFileInfo(file).path());
}

void MainWindow::loadCameraParam()
{
    QString lastOpenDir = m_setting->value("lastOpenDir", DEFAULT_DATA_PATH).toString();
    QString file = QFileDialog::getOpenFileName(this, tr("Open Camera Parameter"), lastOpenDir, "CAM(*.cam)");
    if (file.isEmpty()) return;
    ui->cloudview->loadCameraParam(file.toLocal8Bit());
    m_setting->setValue("lastOpenDir", QFileInfo(file).path());
}

void MainWindow::handleThemeChanged(Setting::Theme theme)
{
    switch (theme) {
    case Setting::Light:
        ui->cloudview->setBackgroundColor(ct::Color::Light);
        break;
    case Setting::Dark:
        ui->cloudview->setBackgroundColor(ct::Color::Dark);
        break;
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
    QMenu menu(this);
    menu.addAction(QIcon(ICON_OPEN), tr("Open"), this, &MainWindow::open, QKeySequence::Open);
    menu.addAction(QIcon(ICON_CLOSE), tr("Close"), this, &MainWindow::close, QKeySequence::Delete);
    menu.addAction(QIcon(ICON_CLEAR), tr("Clear"), this, &MainWindow::clear);
    menu.addAction(QIcon(ICON_SAVE), tr("Save"), this, &MainWindow::save, QKeySequence::Save);
    menu.addAction(QIcon(ICON_RENAME), tr("Rename"), this, &MainWindow::rename, Qt::CTRL + Qt::Key_R);
    menu.addAction(QIcon(ICON_MERGE), tr("Merge"), this, &MainWindow::merge, Qt::CTRL + Qt::Key_M);
    menu.addAction(QIcon(ICON_CLONE), tr("Clone"), this, &MainWindow::clone, Qt::CTRL + Qt::Key_C);
    menu.exec(ui->cloudlist->mapToGlobal(pos));
    menu.clear();
}

void MainWindow::showCloudViewMenu(const QPoint &pos)
{
    QMenu menu(this);
    menu.addAction(QIcon(ICON_RESET), tr("ResetCamera"), ui->cloudview, &ct::CloudView::resetCamera, Qt::Key_R);
    QAction showFPSAction(tr("ShowFPS"));
    showFPSAction.setCheckable(true);
    showFPSAction.setChecked(ui->cloudview->FPSEnable());
    connect(&showFPSAction, &QAction::toggled, this, [=](bool enable){ ui->cloudview->showFPS(enable);});
    menu.addAction(&showFPSAction);
    QAction showAxesAction(tr("ShowAxes"));
    showAxesAction.setCheckable(true);
    showAxesAction.setChecked(ui->cloudview->AxesEnable());
    connect(&showAxesAction, &QAction::toggled, this, [=](bool enable){ ui->cloudview->showAxes(enable); });
    menu.addAction(&showAxesAction);
    menu.addAction(QIcon(ICON_SCREENSHOOT), tr("SaveScreenshot"), this, &MainWindow::saveScreenshot);
    menu.addAction(QIcon(ICON_CAMERAPARAM), tr("LoadCameraParam"), this, &MainWindow::loadCameraParam);
    menu.addAction(QIcon(ICON_CAMERAPARAM), tr("SaveCameraParam"), this, &MainWindow::saveCameraParam);
    menu.exec(ui->cloudview->mapToGlobal(pos));
    menu.clear();
}

void MainWindow::showConsoleMenu(const QPoint &pos)
{
    QMenu menu(this);
    menu.addAction(QIcon(ICON_COPY), tr("Copy"), ui->console, &ct::Console::copy, QKeySequence::Copy);
    menu.addAction(QIcon(ICON_CLEAR), tr("Clear"), ui->console, &ct::Console::clear);
    menu.addAction(QIcon(ICON_SELECTALL), tr("SelectAll"), ui->console, &ct::Console::selectAll, QKeySequence::SelectAll);
    menu.exec(ui->console->mapToGlobal(pos));
    menu.clear();
}

template<class T>
void MainWindow::createLeftDock(const QString& label)
{
    auto iter = m_left_docks.find(label);
    if (iter == m_left_docks.end()) {
        T* dock = new T(this);
        dock->setCloudView(ui->cloudview);
        dock->setCloudList(ui->cloudlist);
        dock->setConsole(ui->console);
        dock->setAttribute(Qt::WA_DeleteOnClose);
        QObject::connect(dock, &QDockWidget::destroyed, this, [=]{ m_left_docks.erase(label); });
        QObject::connect(m_setting, &Setting::changeLanguageEvent, dock, &CustomDock::handleLanguageChanged);
        QObject::connect(dock, &QDockWidget::visibilityChanged, dock, &QDockWidget::setEnabled);
        this->addDockWidget(Qt::LeftDockWidgetArea, dock);
        this->tabifyDockWidget(ui->PropertiesDock, dock);
        m_left_docks.insert(std::make_pair(label, dock));
        dock->setVisible(true);
        dock->raise();
    } else {
        if (iter->second->isEnabled()) {
            iter->second->close();
        } else {
             iter->second->raise();
        }
    }
}

template<class T>
void MainWindow::createRightDock(const QString& label)
{
    auto iter = m_right_docks.find(label);
    if (iter == m_right_docks.end()) {
        T* dock = new T(this);
        dock->setCloudView(ui->cloudview);
        dock->setCloudList(ui->cloudlist);
        dock->setConsole(ui->console);
        dock->setAttribute(Qt::WA_DeleteOnClose);
        QObject::connect(dock, &CustomDock::destroyed, this, [=]{ m_right_docks.erase(label); });
        QObject::connect(m_setting, &Setting::changeLanguageEvent, dock, &CustomDock::handleLanguageChanged);
        QObject::connect(dock, &CustomDock::visibilityChanged, dock, &CustomDock::setEnabled);
        this->addDockWidget(Qt::RightDockWidgetArea, dock);
        if (!m_right_docks.empty()) {
            this->tabifyDockWidget(m_right_docks.rbegin()->second, dock);
        }
        m_right_docks.insert(std::make_pair(label, dock));
        dock->setVisible(true);
        dock->raise();
    } else {
        if (iter->second->isEnabled()) {
            iter->second->close();
        } else {
            iter->second->raise();
        }
    }
}

template<class T>
void MainWindow::createDialog(const QString& label)
{
    auto iter = std::find_if(m_dialogs.begin(), m_dialogs.end(), [=](QDialog* dialog) {
        return dialog->objectName() == label;
    });
    if (iter == m_dialogs.end()) {
        T* dock = new T(this);
        dock->setCloudView(ui->cloudview);
        dock->setCloudList(ui->cloudlist);
        dock->setConsole(ui->console);
        dock->setAttribute(Qt::WA_DeleteOnClose);
        dock->setAttribute(Qt::WA_TranslucentBackground);
        dock->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
        dock->setObjectName(label);
        dock->setToolTip(label);
        QObject::connect(m_setting, &Setting::changeLanguageEvent, dock, &CustomDialog::handleLanguageChanged);
        QObject::connect(dock, &QDockWidget::destroyed, this, [=]{ 
            auto iter = std::find(m_dialogs.begin(), m_dialogs.end(), dock);
            if (iter != m_dialogs.end()) {
                std::for_each(iter + 1, m_dialogs.end(), [=](QDialog* dialog) {
                    dialog->move(QPoint(dialog->pos().x(), dialog->pos().y() - dock->height() - 6));
                });
                m_dialogs.erase(iter);
            }   
        });
        QObject::connect(this, &MainWindow::posChanged, [=](const QPoint &pos){ 
            auto iter = std::find(m_dialogs.begin(), m_dialogs.end(), dock);
            if (iter != m_dialogs.end()) {
                dock->move(QPoint(dock->pos() + pos));
            }   
        });
        int height = 0;
        for (auto dialog : m_dialogs) height += 6 + dialog->height();
        m_dialogs.push_back(dock);
        dock->show();
        dock->move(ui->cloudview->mapToGlobal(QPoint(ui->cloudview->width() - dock->width(), height)));
    } else {
        (*iter)->close();
    }
}

void MainWindow::moveEvent(QMoveEvent* event)
{
    emit posChanged(event->pos() - event->oldPos());
    return QMainWindow::moveEvent(event);
}
