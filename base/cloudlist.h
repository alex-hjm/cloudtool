/**
 * @file cloudlist.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-29
 */
#ifndef __BASE_CLOUDLIST_H__
#define __BASE_CLOUDLIST_H__

#include "cloud.h"

#include <QListWidget>
#include <QMutex>
#include <QSettings>

CT_BEGIN_NAMESPACE

class CT_EXPORT CloudFileIO : public QObject 
{
  Q_OBJECT
public slots:
    void loadCloudFile(const std::string& file_name);
    void saveCloudFile(const Cloud::Ptr& cloud, const std::string& file_name, bool isBinary);

signals:
    void loadCloudResult(bool res, const Cloud::Ptr& cloud);
    void saveCloudResult(bool res);
};

class CT_EXPORT CloudList : public QListWidget
{
  Q_OBJECT
public:
    explicit CloudList(QWidget* parent = nullptr);

public:
    bool loadClouds();
    inline bool addCloud(const Cloud::Ptr& cloud) { return insertCloud(count(), cloud); }
    bool insertCloud(int index, const Cloud::Ptr& cloud);
    bool removeCloud(const Cloud::Ptr& cloud);
    bool saveCloud(const Cloud::Ptr& cloud);
    Cloud::Ptr mergeCloud(const std::vector<Cloud::Ptr>& clouds);
    Cloud::Ptr cloneCloud(const Cloud::Ptr& cloud);
    Cloud::Ptr renameCloud(const Cloud::Ptr& cloud);

    bool setCloudChecked(const Cloud::Ptr& cloud, bool checked = true);
    void setCloudSelected(const Cloud::Ptr& cloud, bool selected = true);

    int getIndex(const Cloud::Ptr& cloud) {
        auto items = findItems(cloud->id.c_str(), Qt::MatchFlag::MatchExactly);
        return items.empty() ? -1 : row(items.front());
    }

    inline Cloud::Ptr getCloud(int index) {
        auto it = item(index);
        return it->data(Qt::UserRole).value<Cloud::Ptr>();
    }

    inline std::vector<Cloud::Ptr> getSelectedClouds() {
        auto items = selectedItems();
        std::vector<Cloud::Ptr> clouds;
        for(auto item : items) { 
            clouds.push_back(item->data(Qt::UserRole).value<Cloud::Ptr>());
        }
        return clouds;
    }

    inline std::vector<Cloud::Ptr> getAllClouds() {
        std::vector<Cloud::Ptr> clouds;
        for(int i = 0; i < count(); i++) { 
            clouds.push_back(item(i)->data(Qt::UserRole).value<Cloud::Ptr>());
        }
        return clouds;
    }

    inline void removeSelectedClouds() { 
        auto clouds = getSelectedClouds();
        for (auto cloud : clouds) { removeCloud(cloud); }
    }

    inline void removeAllClouds() { 
        auto clouds = getAllClouds();
        for (auto cloud : clouds) { removeCloud(cloud); }
    }

    inline void saveSelectedClouds() {
        auto clouds = getSelectedClouds();
        for (auto cloud : clouds) { saveCloud(cloud); }
    }

    inline void saveAllClouds() {
        auto clouds = getAllClouds();
        for (auto cloud : clouds) { saveCloud(cloud); }
    }

    inline void mergeSelectedClouds() {
        auto cloud = mergeCloud(getSelectedClouds());
        addCloud(cloud);
    }

    inline void cloneSelectedClouds() {
        auto clouds = getSelectedClouds();
        for (auto cloud : clouds) { addCloud(cloneCloud(cloud)); }
    }

    inline void renameSelectedClouds() {
        auto clouds = getSelectedClouds();
        for (auto cloud : clouds) { addCloud(renameCloud(cloud)); }
    }

signals:
    void logging(LogLevel level, const QString& msg);
    void addCloudEvent(const Cloud::Ptr& cloud);
    void updateCloudEvent(const Cloud::Ptr& cloud);
    void removeCloudEvent(const Cloud::Ptr& cloud);

private:
    void showContextMenu(const QPoint &pos);
    void handleItemSelectionChanged();
    void handleItemChanged(QListWidgetItem *item);
    void handleItemTextChanged(const QString &currentText);

private:
    QMutex m_mutex;
    QSettings m_setting;
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDTREE_H__