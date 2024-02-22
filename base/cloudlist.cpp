/**
 * @file cloudlist.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-29
 */

#include "cloudlist.h"

#include <QMessageBox>
#include <QInputDialog>
#include <QThread>
#include <QPushButton>
#include <QMouseEvent>
#include <QMenu>
#include <QFileInfo>
#include <QFileDialog>
#include <QShortcut>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#define CLOUD_MERGE_PERFIX  "merge"
#define CLOUD_CLONE_PERFIX  "clone"

CT_BEGIN_NAMESPACE

void CloudFileIO::loadCloudFile(const QString& file_name) 
{
    int result = -1;
    QFileInfo fileinfo(file_name);
    Cloud::Ptr cloud(new Cloud);
    if (fileinfo.suffix() == "pcd")
        result = pcl::io::loadPCDFile(file_name.toStdString(), *cloud);
    else if (fileinfo.suffix() == "ply")
        result = pcl::io::loadPLYFile(file_name.toStdString(), *cloud);
    if (-1 == result)  {
        loadCloudResult(false, nullptr);
        return;
    }
    cloud->setId(fileinfo.baseName());
    cloud->setPath(fileinfo.absolutePath());
    cloud->updateBBox();
    loadCloudResult(true, cloud);   
}

void CloudFileIO::saveCloudFile(const Cloud::Ptr& cloud, const QString& file_name, bool isBinary)
{
    QFileInfo fileinfo(file_name);
    int result = -1;
    if (fileinfo.suffix() == "pcd")
        result = pcl::io::savePCDFile(file_name.toStdString(), *cloud, isBinary);
    else if (fileinfo.suffix() == "ply")
        result = pcl::io::savePLYFile(file_name.toStdString(), *cloud, isBinary);
    else
        result = pcl::io::savePLYFile(file_name.toStdString() + ".ply", *cloud, isBinary);
    if (-1 == result)  {
        saveCloudResult(false);
        return;
    }
    cloud->setPath(fileinfo.absolutePath());
    saveCloudResult(true);
}

CloudList::CloudList(QWidget* parent) : QListWidget(parent), m_setting(PROJECT_NAME, PROJECT_NAME)
{
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setContextMenuPolicy(Qt::CustomContextMenu);
    setDragDropMode(QAbstractItemView::InternalMove);
    connect(this, &QListWidget::itemChanged, this, &CloudList::handleItemChanged);
    connect(this, &QListWidget::itemSelectionChanged, this, &CloudList::handleItemSelectionChanged);
    connect(this, &QListWidget::currentTextChanged, this, &CloudList::handleItemTextChanged);
    connect(this, &QListWidget::customContextMenuRequested, this, &CloudList::showContextMenu);

    //shotcut
    connect(new QShortcut(QKeySequence::Open, this), &QShortcut::activated, this, &CloudList::loadCloud);
    connect(new QShortcut(QKeySequence::Delete, this), &QShortcut::activated, this, &CloudList::removeSelectedClouds);
    connect(new QShortcut(QKeySequence::Save, this), &QShortcut::activated, this, &CloudList::saveSelectedClouds);
}

void CloudList::loadCloud()
{
    QString filter = "all(*.*);;ply(*.ply);;pcd(*.pcd)";
    QString lastOpenDir = m_setting.value("lastOpenDir", DEFAULT_DATA_PATH).toString();
    QStringList fileList = QFileDialog::getOpenFileNames(this, "Open file", lastOpenDir, filter);
    for(auto file : fileList) this->loadCloudFile(file);
}

bool CloudList::appendCloud(const Cloud::Ptr& cloud)
{
    return insertCloud(count(), cloud);
}

bool CloudList::insertCloud(int index, const Cloud::Ptr& cloud)
{
    if (index < 0 || index > count()) {
        logging(LOG_ERROR, tr("Invalid index: %1").arg(index));
        return false;
    }

    if (cloud == nullptr) {
        logging(LOG_ERROR, tr("Cloud is nullptr"));
        return false;
    }

    if (!findItems(cloud->id(), Qt::MatchFlag::MatchExactly).empty()) {
        int k = QMessageBox::warning(this, "WARNING", "The cloud id already exists.\n Rename it?", QMessageBox::Yes, QMessageBox::Cancel);
        if(k == QMessageBox::Cancel) return false;
        bool ok = false;
        QString res = QInputDialog::getText(this, PROJECT_NAME, "Please input cloud id: ", QLineEdit::Normal, cloud->id(), &ok);
        if (!ok)  return false;
        if (res == cloud->id() || !findItems(res, Qt::MatchFlag::MatchExactly).empty()) {
            logging(LOG_ERROR, tr("The cloud id: %1 already exists, Please retry.").arg(res));
            return false;
        }
        cloud->setId(res);
    }

    QListWidgetItem* item = new QListWidgetItem(this);
    item->setText(cloud->id());
    item->setCheckState(Qt::Checked);
    item->setData(Qt::UserRole, QVariant::fromValue(cloud));
    item->setToolTip(cloud->path());
    item->setFlags(item->flags() | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);
    insertItem(index, item);
    setCurrentItem(item);
    addCloudEvent(cloud);
    return true;
}

bool CloudList::removeCloud(const Cloud::Ptr& cloud)
{
    auto item = takeItem(getIndex(cloud));
    if (item == nullptr) return false;
    delete item;
    removeCloudEvent(cloud->id());
    return true;
}

bool CloudList::saveCloud(const Cloud::Ptr& cloud)
{
    QString filter = "ply(*.ply);;pcd(*.pcd)";
    QString lastOpenDir = m_setting.value("lastOpenDir", DEFAULT_DATA_PATH).toString();
    QString filePath = QFileDialog::getSaveFileName(this, tr("Save file"), lastOpenDir + "/" + cloud->id(), filter);
    if (filePath.isEmpty()) return false;
    QMessageBox messageBox(QMessageBox::NoIcon, PROJECT_NAME, tr("Save in binary or ascii format"),
                            QMessageBox::NoButton, this);
    messageBox.addButton(tr("Ascii"), QMessageBox::ActionRole);
    messageBox.addButton(tr("Binary"), QMessageBox::ActionRole)->setDefault(true);
    messageBox.addButton(tr("Close"), QMessageBox::RejectRole)->setHidden(true);
    int isBinary = messageBox.exec();
    if (2 == isBinary) return false;
    saveCloudFile(cloud, filePath, isBinary);
    return true;
}

bool CloudList::mergeClouds(const std::vector<Cloud::Ptr>& clouds)
{
    if (clouds.size() < 2) return false;
    Cloud::Ptr merged_cloud(new Cloud(CLOUD_MERGE_PERFIX));
    for(auto& cloud : clouds) {
        *merged_cloud += *cloud;
        merged_cloud->setId(merged_cloud->id() + "_" + cloud->id());
    }
    return appendCloud(merged_cloud);
}

bool CloudList::cloneCloud(const Cloud::Ptr& cloud)
{
    Cloud::Ptr cloned_cloud = cloud->makeShared();
    cloned_cloud->setId(QString(CLOUD_CLONE_PERFIX) + "_" + cloud->id());
    return appendCloud(cloned_cloud);
}

bool CloudList::renameCloud(const Cloud::Ptr& cloud)
{
    bool ok = false;
    QString id = QInputDialog::getText(this, PROJECT_NAME, "Rename: ", QLineEdit::Normal, cloud->id(), &ok);
    if (!ok)  return false;
    return renameCloud(cloud, id);
}

bool CloudList::renameCloud(const Cloud::Ptr& cloud, const QString& id)
{
    auto it = item(getIndex(cloud));
    if (it == nullptr) return false;
    removeCloudEvent(cloud->id());
    cloud->setId(id);
    it->setText(id);
    addCloudEvent(cloud);
    return true;
}

Cloud::Ptr CloudList::getCloud(int index) const
{
    if (index < 0 || index > count()) return nullptr;
    return item(index)->data(Qt::UserRole).value<Cloud::Ptr>();
}

int CloudList::getIndex(const Cloud::Ptr& cloud) const
{
    for(int i = 0; i < count(); i++) { 
        if (cloud == getCloud(i)) return i;
    }
    return -1;
}

std::vector<Cloud::Ptr> CloudList::getSelectedClouds() const
{
    auto items = selectedItems();
    std::vector<Cloud::Ptr> clouds;
    for(auto& item : items) { 
        clouds.push_back(item->data(Qt::UserRole).value<Cloud::Ptr>());
    }
    return clouds;
}

std::vector<Cloud::Ptr> CloudList::getAllClouds() const
{
    std::vector<Cloud::Ptr> clouds;
    for(int i = 0; i < count(); i++) { 
        clouds.push_back(getCloud(i));
    }
    return clouds;
}

void CloudList::loadCloudFile(const QString& file)
{
    QThread *thread = new QThread;
    CloudFileIO *fileio = new CloudFileIO;
    fileio->moveToThread(thread);
    connect(thread, &QThread::finished, fileio, &QObject::deleteLater);
    connect(thread, &QThread::started, [=] { fileio->loadCloudFile(file); });
    connect(fileio, &CloudFileIO::loadCloudResult, this, [=](bool res, const Cloud::Ptr& cloud) {
        QMutexLocker locker(&m_mutex);
        if (!res || !appendCloud(cloud)) {
            logging(LOG_ERROR, tr("Failed to load cloud file: %1").arg(file));
        } else {
            logging(LOG_INFO, tr("Success to load cloud file: %1").arg(file));
        }
        thread->quit();
        thread->wait();
    }, Qt::QueuedConnection);
    thread->start();
    m_setting.setValue("lastOpenDir", QFileInfo(file).path());
}

void CloudList::saveCloudFile(const Cloud::Ptr& cloud, const QString& file, bool isBinary)
{
    QThread *thread = new QThread;
    CloudFileIO *fileio = new CloudFileIO;
    fileio->moveToThread(thread);
    connect(thread, &QThread::finished, fileio, &QObject::deleteLater);
    connect(thread, &QThread::started, [=] { fileio->saveCloudFile(cloud, file, isBinary);});
    connect(fileio, &CloudFileIO::saveCloudResult, this, [=](bool res) {
        QMutexLocker locker(&m_mutex);
        if (!res || !renameCloud(cloud, QFileInfo(file).baseName())) {
            logging(LOG_ERROR, tr("Failed to save cloud file: %1").arg(file));
        } else {
            logging(LOG_INFO, tr("Success to save cloud file: %1").arg(file));
        }
        thread->quit();
        thread->wait();
    }, Qt::QueuedConnection);
    thread->start();
    m_setting.setValue("lastOpenDir", QFileInfo(file).path());
}

void CloudList::showContextMenu(const QPoint &pos)
{
    QMenu* contextMenu(new QMenu(this));
    auto selectedItems = this->selectedItems();
    contextMenu->addAction("Load", this, &CloudList::loadCloud, QKeySequence::Open);
    contextMenu->addAction("Clear", this, &CloudList::removeAllClouds);
    if (!selectedItems.isEmpty()) {
        contextMenu->addAction("Remove", this, &CloudList::removeSelectedClouds, QKeySequence::Delete);
        contextMenu->addAction("Clone", this, &CloudList::cloneSelectedClouds);
        if (selectedItems.size() == 1) {
            contextMenu->addAction("Save", this, &CloudList::saveSelectedClouds, QKeySequence::Save);
            contextMenu->addAction("Rename", this, &CloudList::renameSelectedClouds);
        } else {
            contextMenu->addAction("Merge", this, &CloudList::mergeSelectedClouds);
        }
    } 
    contextMenu->exec(mapToGlobal(pos));
}

void CloudList::handleItemSelectionChanged()
{
    for(int i = 0; i < count(); i++) { 
        if (item(i)->isSelected()) {
            addCloudBBoxEvent(getCloud(i));
        } else {
            removeCloudBBoxEvent(getCloud(i)->bboxId());
        }
    }

    auto clouds = getSelectedClouds();
    selectCloudEvent(clouds.size() > 0 ? clouds.front() : nullptr);
}

void CloudList::handleItemChanged(QListWidgetItem *item)
{
    auto cloud = getCloud(row(item));
    if (cloud == nullptr) return;
    if (item->checkState() == Qt::Checked) {
        addCloudEvent(cloud);
    } else if (item->checkState() == Qt::Unchecked) {
        removeCloudEvent(cloud->id());
    }
}

void CloudList::handleItemTextChanged(const QString &currentText)
{
    auto cloud = getCloud(currentRow());
    if (cloud == nullptr) return;
    this->renameCloud(cloud, currentText);
}

CT_END_NAMESPACE
