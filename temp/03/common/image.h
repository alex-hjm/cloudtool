#ifndef IMAGE_H
#define IMAGE_H
#include <QDateTime>
#include <QFileInfo>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class Image:public cv::Mat
{
public:
    Image()=default;

    std::string id="";
    QFileInfo info;
    using Ptr = std::shared_ptr<Image>;
    using ConstPtr = std::shared_ptr<const Image>;

    inline QString path() const { return info.path(); }
    inline QString type() {return info.suffix();}
    inline QDateTime birthTime() {return info.birthTime();}
    inline float fileSize() const { return info.size()/1024; }
    inline void setId(const std::string& _id) { id=_id;}
    inline void setInfo(const QFileInfo& _info) { info=_info;id=_info.baseName().toStdString();}
    inline void rename(const std::string& name) { setId(name);}
    inline void prefix(const std::string& pre) { setId(pre+id);}
    inline void suffix(const std::string& suf) { setId(id+suf);}
    inline void copy(const cv::Mat& image) {image.copyTo(*this);}
    inline Ptr makeShared() const { return Ptr (new Image(*this)); }
};

#endif // IMAGE_H
