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
#include <QMenu>

CT_BEGIN_NAMESPACE

class CT_EXPORT CloudFileIO : public QObject 
{
  Q_OBJECT
public slots:
    void loadCloudFile(const QString& file_name);
    void saveCloudFile(const Cloud::Ptr& cloud, const QString& file_name, bool isBinary); 

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
    void loadCloud();

    bool appendCloud(const Cloud::Ptr& cloud);
    bool insertCloud(int index, const Cloud::Ptr& cloud);
    bool removeCloud(const Cloud::Ptr& cloud);
    bool saveCloud(const Cloud::Ptr& cloud);
    bool mergeClouds(const std::vector<Cloud::Ptr>& clouds);
    bool cloneCloud(const Cloud::Ptr& cloud);
    bool renameCloud(const Cloud::Ptr& cloud);
    bool renameCloud(const Cloud::Ptr& cloud, const QString& id);
    
    Cloud::Ptr getCloud(int index) const;
    int getIndex(const Cloud::Ptr& cloud) const;

    std::vector<Cloud::Ptr> getSelectedClouds() const;
    std::vector<Cloud::Ptr> getAllClouds() const;

    inline void removeSelectedClouds() {
        for (auto& cloud : getSelectedClouds()) removeCloud(cloud);
    }

    inline void removeAllClouds() { 
        for (auto& cloud : getAllClouds()) removeCloud(cloud);
    }

    inline void saveSelectedClouds() {
        for (auto& cloud : getSelectedClouds()) saveCloud(cloud); 
    }

    inline void saveAllClouds() {
        for (auto& cloud : getAllClouds()) saveCloud(cloud); 
    }

    inline void mergeSelectedClouds() {
        mergeClouds(getSelectedClouds());
    }

    inline void cloneSelectedClouds() {
        for (auto& cloud : getSelectedClouds()) cloneCloud(cloud); 
    }

    inline void renameSelectedClouds() {
        for (auto& cloud : getSelectedClouds()) renameCloud(cloud); 
    }

private:
    void loadCloudFile(const QString& file);
    void saveCloudFile(const Cloud::Ptr& cloud, const QString& file, bool isBinary);
    
signals:
    void logging(LogLevel level, const QString& msg);
    void addCloudEvent(const Cloud::Ptr& cloud);
    void removeCloudEvent(const QString& id);
    void addCloudBBoxEvent(const Cloud::Ptr& cloud);
    void removeCloudBBoxEvent(const QString& id);
    void selectCloudEvent(const Cloud::Ptr& cloud);

private:
    void showContextMenu(const QPoint &pos);
    void handleItemSelectionChanged();
    void handleItemChanged(QListWidgetItem *item);
    void handleItemTextChanged(const QString &currentText);

private:
    QMutex m_mutex;
    QSettings m_setting;
    QMenu m_menu;
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDTREE_H__