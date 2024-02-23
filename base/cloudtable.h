/**
 * @file cloudtable.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-29
 */
#ifndef __BASE_CLOUDTABLE_H__
#define __BASE_CLOUDTABLE_H__

#include "cloud.h"

#include <QTableWidget>


CT_BEGIN_NAMESPACE

class CT_EXPORT CloudTable : public QTableWidget
{
  Q_OBJECT
public:
    explicit CloudTable(QWidget* parent = nullptr);

signals:
    void updateCloudEvent(const Cloud::Ptr& cloud);

public slots:
    void handleSelectCloud(const Cloud::Ptr& cloud);

private:
    QStringList m_properties;
    QStringList m_labels;
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDTABLE_H__