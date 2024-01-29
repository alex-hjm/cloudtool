/**
 * @file cloudtree.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-29
 */

#include "cloudtree.h"

#include <QMouseEvent>
#include <QMenu>
#include <QFileInfo>
#include <QFileDialog>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

CT_BEGIN_NAMESPACE

CloudTree::CloudTree(QWidget* parent) : QTreeWidget(parent)
{
  
}

bool CloudTree::LoadCloudFile(const std::string& file, Cloud::Ptr& cloud)
{
    int result = -1;
    QFileInfo info(file.c_str());
    if (info.suffix() == "pcd")
        result = pcl::io::loadPCDFile(file, *cloud);
    else if (info.suffix() == "ply")
        result = pcl::io::loadPLYFile(file, *cloud);
    else
        return false;
    if (-1 == result) return false;
    cloud->id = info.baseName().toStdString();
    cloud->path = info.filePath().toStdString();
    return true;
}

bool CloudTree::SaveCloudFile(const Cloud::Ptr& cloud, const std::string& file, bool is_binary)
{
    QFileInfo fileinfo(file.c_str());
    int result = -1;
    if (fileinfo.suffix() == "pcd")
        result = pcl::io::savePCDFile(file, *cloud, is_binary);
    else if (fileinfo.suffix() == "ply")
        result = pcl::io::savePLYFile(file, *cloud, is_binary);
    else
        result = pcl::io::savePLYFile(file + ".ply", *cloud, is_binary);
    if(-1 == result) return false;
    return true;
}

bool CloudTree::loadCloud()
{
    QString filter = "all(*.*);;ply(*.ply);;pcd(*.pcd)";
    QStringList pathList = QFileDialog::getOpenFileNames(this, "Open file", "", filter);
    for(auto path : pathList) {
        Cloud::Ptr cloud(new Cloud);
        if (!CloudTree::LoadCloudFile(path.toStdString(), cloud)) {
            logging(LOG_ERROR, tr("Load pointcloud[path:%1] failed.").arg(path));
            continue;
        }
        if(!this->appendCloud(cloud)) continue;
        logging(LOG_INFO, tr("Load pointcloud[path:%1] success. ").arg(path));
    } 
    return true;
}

bool CloudTree::appendCloud(const Cloud::Ptr& cloud)
{
   return true; 
}

bool CloudTree::removeSelectedClouds()
{
    return true;
}

bool CloudTree::removeAllClouds()
{
    return true;
}

bool CloudTree::mergeSelectedClouds()
{
    return true;
}

bool CloudTree::cloneSelectedClouds()
{
    return true;
}

void CloudTree::mousePressEvent(QMouseEvent* event)
{
    QModelIndex indexSelect = indexAt(event->pos());
    if (event->button() == Qt::RightButton) {
        QMenu* menu = new QMenu(this);
        if(indexSelect.row() != -1) {
            QTreeWidgetItem* item = this->itemFromIndex(indexSelect);
            if (!item->isSelected()) return;
            menu->addAction("remove", [=] { this->removeSelectedClouds(); });
            menu->addAction("clone", [=] { this->cloneSelectedClouds(); });
            auto items = this->selectedItems();
            if(items.size() > 1)
                menu->addAction("merge", [=] { this->mergeSelectedClouds(); });
        } else {
            menu->addAction("load", [=] { this->loadCloud();});
            menu->addAction("clear", [=] { this->removeAllClouds();});
        }
        menu->exec(event->globalPos());
        event->accept();
    }
    return QTreeWidget::mouseReleaseEvent(event);
}


CT_END_NAMESPACE
