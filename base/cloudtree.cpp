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
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setColumnCount(1);
    setHeaderHidden(true);
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, &QTreeWidget::itemChanged, this, &CloudTree::handleItemChanged);
    connect(this, &QTreeWidget::itemSelectionChanged, this, &CloudTree::handleItemSelectionChanged);
    connect(this, &QTreeWidget::customContextMenuRequested, this, &CloudTree::showContextMenu);
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
    cloud->path = info.path().toStdString();
    return true;
}

bool CloudTree::SaveCloudFile(const Cloud::Ptr& cloud, const std::string& file, bool isBinary)
{
    QFileInfo fileinfo(file.c_str());
    int result = -1;
    if (fileinfo.suffix() == "pcd")
        result = pcl::io::savePCDFile(file, *cloud, isBinary);
    else if (fileinfo.suffix() == "ply")
        result = pcl::io::savePLYFile(file, *cloud, isBinary);
    else
        result = pcl::io::savePLYFile(file + ".ply", *cloud, isBinary);
    if(-1 == result) return false;
    return true;
}

bool CloudTree::loadCloud()
{
    QString filter = "all(*.*);;ply(*.ply);;pcd(*.pcd)";
    //QStringList pathList = QFileDialog::getOpenFileNames(this, "Open file", "", filter);
    QStringList pathList{"D:/Project/VSCode/cloudtool/data/rabbit.pcd"};
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
    QTreeWidgetItem* parent(new QTreeWidgetItem(this));
    parent->setText(0, cloud->path.c_str());
    parent->setCheckState(0, Qt::Checked);

    QTreeWidgetItem* item(new QTreeWidgetItem(parent));
    item->setText(0, cloud->id.c_str());
    item->setData(0, Qt::UserRole, QVariant::fromValue(cloud));
    item->setCheckState(0, Qt::Checked);

    QTreeWidgetItem* itema(new QTreeWidgetItem(parent));
    itema->setText(0, cloud->id.c_str());
    itema->setData(0, Qt::UserRole, QVariant::fromValue(cloud));
    itema->setCheckState(0, Qt::Checked);

    QTreeWidgetItem* itemb(new QTreeWidgetItem(parent));
    itemb->setText(0, cloud->id.c_str());
    itemb->setData(0, Qt::UserRole, QVariant::fromValue(cloud));
    itemb->setCheckState(0, Qt::Checked);

    addTopLevelItem(parent);
    expandItem(parent);
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

void CloudTree::showContextMenu(const QPoint &pos)
{
    QMenu* contextMenu(new QMenu(this));

    auto selectedItems = this->selectedItems();

    if (!selectedItems.isEmpty()) {
        contextMenu->addAction("remove", [=] { this->removeSelectedClouds(); });
        contextMenu->addAction("clone", [=] { this->cloneSelectedClouds(); });
        if (selectedItems.size() == 1 && selectedItems.first()->isSelected()) {
            // TODO:
        } else {
            contextMenu->addAction("merge", [=] { this->mergeSelectedClouds(); });
        }
    } else {
        contextMenu->addAction("load", [=] { this->loadCloud();});
        contextMenu->addAction("clear", [=] { this->removeAllClouds();});
    }
     contextMenu->exec(mapToGlobal(pos));
}

void CloudTree::handleItemSelectionChanged()
{
    auto items = selectedItems();
    for (auto item : items) {
        updateChildItems(item);
        updateParentItem(item);
    }
}

void CloudTree::handleItemChanged(QTreeWidgetItem *item, int column)
{
    if (item != nullptr && column == 0) {
        updateChildItems(item);
        updateParentItem(item);
    }
}

void CloudTree::updateChildItems(QTreeWidgetItem *parentItem)
{
    for (int i = 0; i < parentItem->childCount(); ++i) {
        QTreeWidgetItem *childItem = parentItem->child(i);
        blockSignals(true);
        childItem->setCheckState(0, parentItem->checkState(0));
        childItem->setSelected(parentItem->isSelected());
        blockSignals(false);
        updateChildItems(childItem);
    }
}

void CloudTree::updateParentItem(QTreeWidgetItem *childItem)
{
    QTreeWidgetItem *parentItem = childItem->parent();
    if (parentItem != nullptr) {
        int checkedCount = 0;
        int selectedCount = 0;

        for (int i = 0; i < parentItem->childCount(); ++i) {
            if (parentItem->child(i)->checkState(0) == Qt::Checked) {
                checkedCount++;
            }
        
            if (parentItem->child(i)->isSelected()) {
                selectedCount++;
            }
        }

        blockSignals(true);
        if (checkedCount == parentItem->childCount()) {
            parentItem->setCheckState(0, Qt::Checked);
        } else if (checkedCount == 0) {
            parentItem->setCheckState(0, Qt::Unchecked);
        } else {
            parentItem->setCheckState(0, Qt::PartiallyChecked);
        }

        if (selectedCount == parentItem->childCount()) {
            parentItem->setSelected(true);
        } else {
            parentItem->setSelected(false);
        }
        blockSignals(false);
        updateParentItem(parentItem);
    }
}


CT_END_NAMESPACE
