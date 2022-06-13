#include "fileio.h"

FileIO::FileIO(QObject *parent) : QObject(parent)
{

}

void FileIO::loadPointCloud(const QString& path)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud(new Cloud);
    //QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=path.toLocal8Bit().toStdString();
    boost::filesystem::path p (file_name.c_str());
    std::string extension = p.extension ().string ();
    int result = -1;
    if (extension == ".pcd")
        result = pcl::io::loadPCDFile (file_name, *cloud);
    else if (extension == ".ply")
        result = pcl::io::loadPLYFile (file_name, *cloud);
    else {
        emit loadCloudResult(false,cloud,time.toc());
        return;
    }
    if(result==-1){
        emit loadCloudResult(false,cloud,time.toc());
        return;
    }
    //remove NaN
    cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
    //init
    cloud->setInfo(QFileInfo(path));
    cloud->update();
    emit loadCloudResult(true,cloud,time.toc());
}

void FileIO::savePointCloud(const Cloud::Ptr& cloud,const QString& path,bool isBinary)
{
    pcl::console::TicToc time;
    time.tic();
    //QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=path.toLocal8Bit().toStdString();
    boost::filesystem::path p (file_name.c_str());
    std::string extension = p.extension ().string ();
    int result = -1;
    if (extension == ".pcd")
        result = pcl::io::savePCDFile (file_name, *cloud,isBinary);
    else if (extension == ".ply")
        result = pcl::io::savePLYFile (file_name, *cloud,isBinary);
    else{
        file_name.append(".ply");
        result = pcl::io::savePLYFile (file_name, *cloud,isBinary);
    }
    if(result==-1)
        emit saveCloudResult(false,QString::fromStdString(file_name),time.toc());
    else
        emit saveCloudResult(true,QString::fromStdString(file_name),time.toc());
}

bool FileIO::loadImage(Image::Ptr &image,const QString &path)
{
    //QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=path.toLocal8Bit().toStdString();
    image->copy(cv::imread(file_name));
    if(image->empty())
        return false;
    image->setInfo(QFileInfo(path));
    return true;
}

bool FileIO::saveImage(const Image::Ptr &image, const QString &path)
{
    //QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=path.toLocal8Bit().toStdString();
    if(!cv::imwrite(file_name,*image))
        return false;
    return true;
}
