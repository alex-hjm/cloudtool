/**
 * @file fileio.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_FILEIO_H
#define CT_BASE_FILEIO_H

#include "base/cloud.h"
#include "base/exports.h"

namespace ct
{
    class CT_EXPORT FileIO : public QObject
    {
        Q_OBJECT
    public:
        explicit FileIO(QObject *parent = nullptr) : QObject(parent) {}

    signals:
        /**
         * @brief 加载点云文件的结果
         */
        void loadCloudResult(bool success, const Cloud::Ptr &cloud, float time);

        /**
         * @brief 保存点云文件的结果
         */
        void saveCloudResult(bool success, const QString &filename, float time);

    public slots:
        /**
         * @brief 加载点云文件
         */
        void loadPointCloud(const QString &filename);

        /**
         * @brief 保存点云文件
         */
        void savePointCloud(const Cloud::Ptr &cloud, const QString &filename, bool isBinary);
    };
}

#endif