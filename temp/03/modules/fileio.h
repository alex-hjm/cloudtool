#ifndef FILEIO_H
#define FILEIO_H
#include <QFileInfo>
#include <QTextCodec>
#include <QMutex>
#include <pcl/console/time.h>
#include <pcl/io/auto_io.h>
#include <pcl/filters/filter.h>
#include "common/cloud.h"
#include "common/image.h"

class FileIO :public QObject
{
     Q_OBJECT
public:
    explicit FileIO(QObject *parent = nullptr);

signals:
    void loadCloudResult(bool success,const Cloud::Ptr &cloud,float time);
    void saveCloudResult(bool success,const QString& name,float time);

public slots:
    void loadPointCloud(const QString& path);
    void savePointCloud(const Cloud::Ptr &cloud,const QString& path,bool isBinary);
    static bool loadImage(Image::Ptr &image,const QString& path);
    static bool saveImage(const Image::Ptr &image,const QString& path);
};

#endif // FILEIO_H
