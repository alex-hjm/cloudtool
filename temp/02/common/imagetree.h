#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>

#include "customtree.h"
#include "graphicsview.h"
#include "console.h"
#include "src/common/image.h"
#include "src/modules/fileio.h"

class ImageTree : public CustomTree
{
    Q_OBJECT

public:
    explicit ImageTree(QWidget *parent = nullptr);

    Console *console;
    GraphicsView *graphicsView;
    QTableWidget *imageTable;

    bool loadImage(int row,const QString &path);
    void addImages();
    void insertImage(int row,Image &image,const bool&selected);
    void updateImage(const Index &index,Image &updateimage);
    void updateSelectedImages(std::vector<Image>&updateimages);
    void clearImage(const Index &index);
    void clearSelectedImages();
    void clearAllImages();
    void setImageChecked(const Index &index,bool checked);
    void setSelectedImagesChecked(bool checked);
    void setImageSelected(const Index &index,bool selected);
    void setSelectedImagesSelected(bool selected);
    void saveSelectedImages();
    void cloneSelectedImages();

    inline size_t size(){return imageVec.size();}
    Image getImage(const Index &index);
    Image getSelectedImage();
    std::vector<Image> getCheckedImages();
    std::vector<Image> getSelectedImages();
    std::vector<Image> getAllImages();
    void setAcceptDrops(const bool &enable);
    void setExtendedSelection(const bool &enable);

public slots:
    void updateProperty();
    void updateScreen(QTreeWidgetItem *, int);

private:

    FileIO fileio;
    Index index;
    QFileInfo lastFileInfo;
    std::vector<Image> selectedImages;
    std::vector<Image> clickedImages;
    std::vector<std::vector<Image>> imageVec;
};

#endif // IMAGEDATA_H
