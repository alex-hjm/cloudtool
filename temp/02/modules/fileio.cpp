#include "fileio.h"

FileIO::FileIO()
{

}

bool
FileIO::loadPointCloud(CloudXYZRGBN::Ptr &cloud, const QFileInfo &fileInfo)
{
    time.tic();
    cloud.reset(new CloudXYZRGBN) ;
    QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=pcode->fromUnicode(fileInfo.filePath()).data();
    boost::filesystem::path p (file_name.c_str());
    std::string extension = p.extension ().string ();
    int result = -1;
    if (extension == ".pcd")
        result = pcl::io::loadPCDFile (file_name, *cloud);
    else if (extension == ".ply")
        result = pcl::io::loadPLYFile (file_name, *cloud);
    else if (extension == ".ifs")
        result = pcl::io::loadIFSFile (file_name, *cloud);
    else if (extension == ".obj")
        result = pcl::io::loadOBJFile (file_name, *cloud);
    else
        return false;
    if(result!=-1) {
        if (!cloud->is_dense) {
            cloud->is_dense = false;
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
        }
        tocTime=time.toc();
        return true;
    }
    else
        return false;
}

bool
FileIO::savePointCloud(CloudXYZRGBN::Ptr &cloud, const QFileInfo &fileInfo,bool isBinary)
{
    QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=pcode->fromUnicode(fileInfo.filePath()).data();
    boost::filesystem::path p (file_name.c_str());
    std::string extension = p.extension ().string ();
    time.tic();
    int result = -1;
    if (extension == ".pcd")
        result = pcl::io::savePCDFile (file_name, *cloud,isBinary);
    else if (extension == ".ply")
        result = pcl::io::savePLYFile (file_name, *cloud,isBinary);
    else if (extension == ".ifs")
        result = pcl::io::saveIFSFile (file_name, *cloud);
    else
    {
        file_name.append(".ply");
        result = pcl::io::savePLYFile (file_name, *cloud,isBinary);
    }
    tocTime=time.toc();
    if(result!=-1)
        return true;
    else
        return false;

}

void FileIO::savePointCloud(CloudXYZRGBN::Ptr &cloud, const QFileInfo &fileInfo)
{
    QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=pcode->fromUnicode(fileInfo.filePath()).data();
    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply"<< std::endl;
    ofs << "format binary_little_endian 1.0" << std::endl;
    ofs << "element vertex" << " " << cloud->size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
//    ofs << "property uchar red" << std::endl;
//    ofs << "property uchar green" << std::endl;
//    ofs << "property uchar blue" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::ofstream ofs_text(file_name, std::ios::app | std::ios::binary);
    for (size_t i = 0; i < cloud->size(); i++) {
        ofs_text.write (reinterpret_cast<const char*> (&(cloud->points[i].x)), sizeof (float));
        ofs_text.write (reinterpret_cast<const char*> (&(cloud->points[i].y)), sizeof (float));
        ofs_text.write (reinterpret_cast<const char*> (&(cloud->points[i].z)), sizeof (float));
//        ofs_text.write (reinterpret_cast<const char*> (&(cloud->points[i].r)), sizeof (std::uint8_t));
//        ofs_text.write (reinterpret_cast<const char*> (&(cloud->points[i].g)), sizeof (std::uint8_t));
//        ofs_text.write (reinterpret_cast<const char*> (&(cloud->points[i].b)), sizeof (std::uint8_t));
    }
    ofs_text.close();
}

bool
FileIO::loadImage(cv::Mat& image, QFileInfo &fileInfo)
{
    time.tic();
    QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=pcode->fromUnicode(fileInfo.filePath()).data();
    image=cv::imread(file_name);
    if(image.empty())
        return false;
    tocTime=time.toc();
    return true;
}

bool
FileIO::saveImage(cv::Mat& image, QFileInfo &fileInfo)
{
    time.tic();
    QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=pcode->fromUnicode(fileInfo.filePath()).data();
    if(!cv::imwrite(file_name,image))
        return false;
    tocTime=time.toc();
    return true;
}
