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

#include <QMouseEvent>
#include <QMenu>
#include <QFileInfo>
#include <QFileDialog>
#include <QRunnable>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

CT_BEGIN_NAMESPACE

void CloudFileIO::loadCloudFile(const std::string& file_name) 
{
    int result = -1;
    QFileInfo info(file_name.c_str());
    Cloud::Ptr cloud(new Cloud);
    if (info.suffix() == "pcd")
        result = pcl::io::loadPCDFile(file_name, *cloud);
    else if (info.suffix() == "ply")
        result = pcl::io::loadPLYFile(file_name, *cloud);
    if (-1 == result)  {
        emit loadCloudResult(false, nullptr);
        return;
    }
    cloud->id = info.baseName().toStdString();
    cloud->path = info.absolutePath().toStdString();
    emit loadCloudResult(true, cloud);
}

void CloudFileIO::saveCloudFile(const Cloud::Ptr& cloud, const std::string& file_name, bool isBinary) 
{
    QFileInfo fileinfo(file_name.c_str());
    int result = -1;
    if (fileinfo.suffix() == "pcd")
        result = pcl::io::savePCDFile(file_name, *cloud, isBinary);
    else if (fileinfo.suffix() == "ply")
        result = pcl::io::savePLYFile(file_name, *cloud, isBinary);
    else
        result = pcl::io::savePLYFile(file_name + ".ply", *cloud, isBinary);
    emit saveCloudResult(bool(result));
}

CloudList::CloudList(QWidget* parent) : QListWidget(parent), m_setting(PROJECT_NAME, PROJECT_NAME)
{
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setContextMenuPolicy(Qt::CustomContextMenu);
    setDragDropMode(QAbstractItemView::InternalMove);
    connect(this, &QListWidget::itemChanged, this, &CloudList::handleItemChanged);
    connect(this, &QListWidget::itemSelectionChanged, this, &CloudList::handleItemSelectionChanged);
    connect(this, &QListWidget::customContextMenuRequested, this, &CloudList::showContextMenu);
    connect(this, &QListWidget::currentTextChanged, this, &CloudList::handleItemTextChanged);
}

bool CloudList::loadClouds()
{
    QString filter = "all(*.*);;ply(*.ply);;pcd(*.pcd)";
    QString lastOpenDir = m_setting.value("lastOpenDir", DEFAULT_DATA_PATH).toString();
    QStringList fileList = QFileDialog::getOpenFileNames(this, "Open file", lastOpenDir, filter);
    for(auto file : fileList) {
        QThread *thread = new QThread;
        CloudFileIO *fileio = new CloudFileIO;
        fileio->moveToThread(thread);
        connect(thread, &QThread::finished, fileio, &QObject::deleteLater);
        connect(thread, &QThread::started, [=] { fileio->loadCloudFile(file.toStdString());});
        connect(fileio, &CloudFileIO::loadCloudResult, this, [=](bool res, const Cloud::Ptr& cloud) {
            QMutexLocker locker(&m_mutex);
            if (!res || !addCloud(cloud)) {
                logging(LOG_ERROR, tr("Failed to load cloud file: %1").arg(file));
                return;
            }
            logging(LOG_INFO, tr("Success to load cloud file: %1").arg(file));
        }, Qt::QueuedConnection);
        thread->start();
        m_setting.setValue("lastOpenDir", QFileInfo(file).path());
    } 
    return true;
}

bool CloudList::insertCloud(int index, const Cloud::Ptr& cloud)
{
    if(!findItems(cloud->id.c_str(), Qt::MatchFlag::MatchExactly).empty()) {
        int k = QMessageBox::warning(this, "WARNING", "The cloud id already exists.\n Rename it?", QMessageBox::Yes, QMessageBox::Cancel);
        if(k == QMessageBox::Cancel) return false;
        bool ok = false;
        QString res = QInputDialog::getText(this, PROJECT_NAME, "Please input cloud id: ", QLineEdit::Normal, cloud->id.c_str(), &ok);
        if (!ok) return false;
        else {
            if (res == cloud->id.c_str() || !findItems(res, Qt::MatchFlag::MatchExactly).empty())
            {
                logging(LOG_ERROR, tr("The cloud id already exists.").arg(res));
                return false;
            }
            cloud->id = res.toStdString();
        }
    }
    QListWidgetItem* item = new QListWidgetItem(this);
    item->setText(cloud->id.c_str());
    item->setCheckState(Qt::Checked);
    item->setData(Qt::UserRole, QVariant::fromValue(cloud));
    item->setToolTip(cloud->id.c_str());
    item->setFlags(item->flags() | Qt::ItemIsDragEnabled);
    insertItem(index, item);
    setCurrentItem(item);
    emit addCloudEvent(cloud);
    return true;
}

bool CloudList::removeCloud(const Cloud::Ptr& cloud)
{
    
    return true;
}

bool CloudList::saveCloud(const Cloud::Ptr& cloud)
{
    return true;
}

Cloud::Ptr CloudList::mergeCloud(const std::vector<Cloud::Ptr>& clouds)
{
    return nullptr;
}

Cloud::Ptr CloudList::cloneCloud(const Cloud::Ptr& cloud)
{
    return nullptr;
}

Cloud::Ptr CloudList::renameCloud(const Cloud::Ptr& cloud)
{
    return nullptr;
}

void CloudList::showContextMenu(const QPoint &pos)
{
    QMenu* contextMenu(new QMenu(this));

    auto selectedItems = this->selectedItems();

    if (!selectedItems.isEmpty()) {
        contextMenu->addAction("remove", [=] {  this->removeSelectedClouds();  });
        contextMenu->addAction("clone", [=] { this->cloneSelectedClouds(); });
        contextMenu->addAction("rename", [=] { this->cloneSelectedClouds(); });
        if (selectedItems.size() == 1 && selectedItems.first()->isSelected()) {
            // TODO: 
        } else {
            contextMenu->addAction("merge", [=] { this->mergeSelectedClouds();  });
        }
    } else {
        contextMenu->addAction("load", [=] { this->loadClouds();});
        contextMenu->addAction("clear", [=] {  this->removeAllClouds(); });
        contextMenu->addAction("sort", [=] {  this->sortItems(); });
    }
     contextMenu->exec(mapToGlobal(pos));
}

void CloudList::handleItemSelectionChanged()
{

}

void CloudList::handleItemChanged(QListWidgetItem *item)
{

}

void CloudList::handleItemTextChanged(const QString &currentText)
{

}

CT_END_NAMESPACE
