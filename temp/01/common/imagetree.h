#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>
#include <QTableWidget>

#include "console.h"
#include "customtree.h"
#include "imageview.h"
#include "modules/fileio.h"

class ImageTree : public CustomTree
{
    Q_OBJECT

public:
    explicit ImageTree(QWidget *parent = nullptr);

    void init(Console* &,ImageView* &,QTableWidget* &);
    void addImages();
    void insertImage(int num,const Image::Ptr &image,const bool&selected);
    void clearImage(const Index &index);
    void clearSelectedImages();
    void clearAllImages();
    void setImageChecked(const Index &index,bool checked);
    void setSelectedImagesChecked(bool checked);
    void setImageSelected(const Index &index,bool selected);
    void saveSelectedImages();
    void cloneSelectedImages();
    void renameSelectedImages();

    Image::Ptr getSelectedImage();
    std::vector<Image::Ptr> getCheckedImages();
    std::vector<Image::Ptr> getSelectedImages();
    std::vector<Image::Ptr> getAllImages();

    void setAcceptDrops(const bool &enable);
    void setExtendedSelection(const bool &enable);

public slots:
    void updatePropertiesTable();
    void updateScreen(QTreeWidgetItem *, int);

protected:
    void mousePressEvent(QMouseEvent *event);
private:
    Console *console;
    ImageView *image_view;
    QTableWidget *image_table;
    std::vector<std::vector<Image::Ptr>> image_vec;

};

#endif // IMAGEDATA_H
