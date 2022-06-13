#ifndef IMAGE_H
#define IMAGE_H
#include <QFileInfo>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class Image
{
public:
    Image()=default;


    Image(const cv::Mat &imagein,const QFileInfo &info):
    image(imagein),fileInfo(info)
    {
        id=fileInfo.baseName().toStdString();
        type=fileInfo.suffix();
    }

    QFileInfo fileInfo;
    std::string id="";
    QString type;
    cv::Mat image;

    inline size_t size() const { return image.total(); } //KB
    inline QString path() const { return fileInfo.path(); }
    inline float fileSize() const { return fileInfo.size()/1024; } //KB
    inline bool empty () const { return image.empty (); }
    inline void rename(const std::string & name) { id=name;fileInfo.setFile(fileInfo.path()+"/"+id.c_str()); }
    inline void append(const std::string & name) { id=id+name;fileInfo.setFile(fileInfo.path()+"/"+id.c_str());}
    inline void prefix(const std::string & name) { id=name+id;fileInfo.setFile(fileInfo.path()+"/"+id.c_str());}
    inline int cols() {return image.cols;}
    inline int rows() {return image.rows;}
};

#endif // IMAGE_H
