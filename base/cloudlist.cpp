/**
 * @file cloudlist.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-29
 */

#include "cloudlist.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

#define CLOUD_MERGE_PERFIX  "merge"
#define CLOUD_CLONE_PERFIX  "clone"

CT_BEGIN_NAMESPACE

CloudList::CloudList(QWidget* parent) : QListWidget(parent)
{
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setContextMenuPolicy(Qt::CustomContextMenu);
    setDragDropMode(QAbstractItemView::InternalMove);
}

bool CloudList::appendCloud(const Cloud::Ptr& cloud)
{
    return insertCloud(count(), cloud);
}

bool CloudList::insertCloud(int index, const Cloud::Ptr& cloud)
{
    if ((cloud == nullptr) || (index < 0 || index > count())) return false;
    int idx = 1;
    QString id = cloud->id();
    while (contains(cloud->id())) {
        cloud->setId(id + "_" + QString::number(idx++));
    }
    QListWidgetItem* item = new QListWidgetItem(this);
    item->setText(cloud->id());
    item->setCheckState(Qt::Checked);
    item->setData(Qt::UserRole, QVariant::fromValue(cloud));
    item->setToolTip(cloud->path());
    item->setFlags(item->flags() | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);
    item->setIcon(m_icon);
    insertItem(index, item);
    clearSelection();
    setCurrentItem(item);
    return true;
}

bool CloudList::removeCloud(const Cloud::Ptr& cloud)
{
    auto item = takeItem(getIndex(cloud));
    if (item == nullptr) return false;
    delete item;
    return true;
}

Cloud::Ptr CloudList::mergeClouds(const std::vector<Cloud::Ptr>& clouds)
{
    if (clouds.size() < 2) return nullptr;
    Cloud::Ptr merged_cloud(new Cloud(CLOUD_MERGE_PERFIX));
    for(auto& cloud : clouds) {
        *merged_cloud += *cloud;
        merged_cloud->setId(merged_cloud->id() + "_" + cloud->id());
    }
    merged_cloud->updateBBox();
    merged_cloud->updateResolution();
    appendCloud(merged_cloud);
    return merged_cloud;
}

Cloud::Ptr CloudList::cloneCloud(const Cloud::Ptr& cloud)
{
    Cloud::Ptr cloned_cloud = cloud->makeShared();
    cloned_cloud->setId(QString(CLOUD_CLONE_PERFIX) + "_" + cloud->id());
    appendCloud(cloned_cloud);
    return cloned_cloud;
}

Cloud::Ptr CloudList::renameCloud(const Cloud::Ptr& cloud, const QString& id)
{
    auto it = item(getIndex(cloud));
    if (it == nullptr) return nullptr;
    cloud->setId(id);
    it->setText(cloud->id());
    return cloud;
}

bool CloudList::contains(const QString& id)
{
    return !findItems(id, Qt::MatchFlag::MatchExactly).empty();
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

CT_END_NAMESPACE
