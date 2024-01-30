/**
 * @file cloudtree.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-29
 */
#ifndef __BASE_CLOUDTREE_H__
#define __BASE_CLOUDTREE_H__

#include "cloud.h"

#include <QTreeWidget>

CT_BEGIN_NAMESPACE

class CT_EXPORT CloudTree : public QTreeWidget
{
  Q_OBJECT
public:
    explicit CloudTree(QWidget* parent = nullptr);

    // io
    static bool LoadCloudFile(const std::string& file, Cloud::Ptr& cloud);
    static bool SaveCloudFile(const Cloud::Ptr& cloud, const std::string& file, bool isBinary);

    bool loadCloud();
    bool saveSelectedClouds();

    bool appendCloud(int row, const Cloud::Ptr& cloud);
    bool insertCloud(int index, const Cloud::Ptr& cloud);
    bool appendCloud(const Cloud::Ptr& cloud);
    bool removeCloud(const Cloud::Ptr& cloud);
    bool removeSelectedClouds();
    bool removeAllClouds();
    bool mergeSelectedClouds();
    bool cloneSelectedClouds();
    bool setCloudChecked(const Cloud::Ptr& cloud, bool checked);

    Cloud::Ptr getCloud(const int index);
    std::vector<Cloud::Ptr> getSelectedClouds();

signals:
    void logging(LogLevel level, const QString& msg);
    void addCloudEvent(const Cloud::Ptr& cloud);
    void removeCloudEvent(const QString& id);
    void removeAllCloudsEvent();
    void removeCloudEvent(const Cloud::Ptr& cloud);

private:
    void showContextMenu(const QPoint &pos);
    void handleItemSelectionChanged();
    void handleItemChanged(QTreeWidgetItem *item, int column);
    void updateChildItems(QTreeWidgetItem *parentItem);
    void updateParentItem(QTreeWidgetItem *childItem);
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDTREE_H__