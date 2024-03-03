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
#include <QIcon>

CT_BEGIN_NAMESPACE

class CT_EXPORT CloudList : public QListWidget
{
  Q_OBJECT
public:
    explicit CloudList(QWidget* parent = nullptr);

public:
    bool appendCloud(const Cloud::Ptr& cloud);
    bool insertCloud(int index, const Cloud::Ptr& cloud);
    bool removeCloud(const Cloud::Ptr& cloud);
    Cloud::Ptr mergeClouds(const std::vector<Cloud::Ptr>& clouds);
    Cloud::Ptr cloneCloud(const Cloud::Ptr& cloud);
    Cloud::Ptr renameCloud(const Cloud::Ptr& cloud, const QString& id);
    bool contains(const QString& id);

    Cloud::Ptr getCloud(int index) const;
    int getIndex(const Cloud::Ptr& cloud) const;

    std::vector<Cloud::Ptr> getSelectedClouds() const;
    std::vector<Cloud::Ptr> getAllClouds() const;

    void setCloudIcon(const QIcon& icon);
private:
    QIcon m_icon;
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDTREE_H__