/**
 * @file fileio.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-02
 */
#ifndef __BASE_FILEIO_H__
#define __BASE_FILEIO_H__

#include "base/cloud.h"
#include <QObject>

CT_BEGIN_NAMESPACE

class CT_EXPORT FileIO : public QObject 
{
  Q_OBJECT
public slots:
    void loadCloudFile(const QString& file_name);
    void saveCloudFile(const Cloud::Ptr& cloud, const QString& file_name, bool isBinary); 

signals:
    void loadCloudResult(bool res, const Cloud::Ptr& cloud);
    void saveCloudResult(bool res);
};

CT_END_NAMESPACE
#endif // __BASE_FILEIO_H__