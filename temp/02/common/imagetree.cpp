#include "imagetree.h"

ImageTree::ImageTree(QWidget *parent):
    CustomTree(parent),
    lastFileInfo(QFileInfo("../"))
{
    connect(this,&ImageTree::itemSelectionChanged,this,&ImageTree::updateProperty);
    connect(this,SIGNAL(itemClicked(QTreeWidgetItem *, int)),this,SLOT(updateScreen(QTreeWidgetItem *, int)));
}

bool  ImageTree::loadImage(int row,const QString &path)
{
    cv::Mat m_image;
    lastFileInfo.setFile(path);
    if(!fileio.loadImage(m_image,lastFileInfo)){
        console->error(tr("Can not open the file!"));
        return false;
    }
    Image image(m_image,lastFileInfo);
    this->insertImage(row,image,false);
    return true;
}

void ImageTree::addImages()
{
    QString filter ="all(*.*);;jpg(*.jpg);;jpg(*.png)";
    QStringList filePathList = QFileDialog::getOpenFileNames(this,tr("Open image file"),lastFileInfo.filePath(),filter);
    if (filePathList.isEmpty()) return;
    console->info(tr("Loading..."));
    float time=0;int num=0;
    for (int i = 0; i != filePathList.size(); i++) {
        if(!loadImage(i,filePathList[i])) continue;
        time+=fileio.tocTime;
        num++;
        cv::waitKey(10);
    }
    console->info(tr("%1 images loaded successfully! Take time :%2 ms.").arg(num).arg(time));
}

void ImageTree::insertImage(int row, Image &image,const bool&selected)
{
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/child2category.svg");
    int currentindex=this->topLevelItemCount()-1;
    if(row==0){
        this->addItem<Image>(-1,image,image.path(),image.id.c_str(),parentIcon,childIcon,selected,imageVec);
        graphicsView->addImage(image.image);
    }
    else{
        this->addItem<Image>(currentindex,image,image.path(),image.id.c_str(),parentIcon,childIcon,selected,imageVec);
        graphicsView->addImage(image.image);
    }

}

void ImageTree::updateImage(const Index &index,Image &updateimage)
{
    this->updateItem<Image>(index,updateimage,imageVec);
    graphicsView->addImage(updateimage.image);
}

void ImageTree::updateSelectedImages(std::vector<Image> &updataimages)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->updateImage(indexs[i],updataimages[i]);
    }
}

void ImageTree::clearImage(const Index &index)
{
    this->removeItem<Image>(index,imageVec);
    graphicsView->removeImage();
}

void ImageTree::clearSelectedImages()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    indexs=this->getSortedIndexs(descending,indexs);
    for(int i=0;i<indexs.size();i++) {
        this->clearImage(indexs[i]);
    }
}

void ImageTree::clearAllImages()
{
    this->imageVec.clear();
    this->clear();
    graphicsView->removeImage();
}

void ImageTree::setImageChecked(const Index &index, bool checked)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    Image image=this->getImage(index);
    this->setItemChecked(index,checked);
    if(indexs.size()<=0)return;
    if(indexs[0]==index)
        if(checked)
            graphicsView->addImage(image.image);
        else
            graphicsView->removeImage();
}

void ImageTree::setSelectedImagesChecked(bool checked)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->setImageChecked(indexs[i],checked);
    }
}

void ImageTree::setImageSelected(const Index &index, bool selected)
{
    this->setItemSelected(index,selected);
    if(selected) {
        this->setImageChecked(index,true);
    }
    else
        graphicsView->removeImage();
}

void ImageTree::setSelectedImagesSelected(bool selected)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->setImageSelected(indexs[i],selected);
    }
}

void ImageTree::saveSelectedImages()
{
    std::vector<Image> selectedImages=this->getSelectedImages();
    for (int i=0;i<selectedImages.size();i++) {
        QString filter ="jpg(*.jpg);;jpg(*.png)";
        QFileInfo fileInfo = QFileDialog::getSaveFileName(this, tr ("Save image"),selectedImages[i].path(), filter);
        if (fileInfo.filePath().isEmpty ())
            continue;

        if(!fileio.saveImage(selectedImages[i].image,fileInfo)) {
            console->error(tr("The image save failed."));
            return;
        }
        console->info(tr("The image ")+ fileInfo.fileName()+tr(" has been saved."));
    }
}

void ImageTree::cloneSelectedImages()
{
    std::vector<Image> selectedImages=this->getSelectedImages();
    std::vector<Index> indexs=this->getSelectedIndexs();
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/child2category.svg");
    for(size_t i=0;i<selectedImages.size();i++) {
        this->addItem<Image>(indexs[i].row,selectedImages[i],selectedImages[i].path(),selectedImages[i].id.c_str(),parentIcon,childIcon,false,imageVec);
        graphicsView->addImage(selectedImages[i].image);
    }
}

Image ImageTree::getImage(const Index &index)
{
    return this->getData<Image>(index,imageVec);
}

Image ImageTree::getSelectedImage()
{
    Image selectedImage;
    if(this->getSelectedImages().size()<=0)
        return selectedImage;
    else
        return this->getSelectedImages()[0];
}

std::vector<Image> ImageTree::getCheckedImages()
{
    return this->getCheckedDatas<Image>(imageVec);
}

std::vector<Image> ImageTree::getSelectedImages()
{
    return this->getSelectedDatas<Image>(imageVec);
}

std::vector<Image> ImageTree::getAllImages()
{
    return this->getAllDatas<Image>(imageVec);
}

void ImageTree::setAcceptDrops(const bool &enable)
{
    if(enable) {
        graphicsView->setAcceptDrops(true);
        connect(graphicsView,&GraphicsView::dropFilePath,[=](QStringList path) {
            console->info(tr("Loading..."));
            float time=0;int num=0;
            for (int i = 0; i != path.size(); i++) {
                if(!loadImage(i,path[i])) continue;
                time+=fileio.tocTime;
                num++;
                cv::waitKey(10);
            }
            console->info(tr("%1 images loaded successfully! Take time :%2 ms.").arg(num).arg(time));
        });
    }
    else {
        graphicsView->setAcceptDrops(false);
        disconnect(graphicsView,&GraphicsView::dropFilePath,0,0);
    }
}

void ImageTree::setExtendedSelection(const bool &enable)
{
    if(enable)
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        this->setSelectionMode(QAbstractItemView::SingleSelection);
}


void ImageTree::updateProperty()
{
    std::vector<Image> selectedImages=this->getSelectedImages();
    QString id,type,resolution,fileSize;
    auto items = this->selectedItems();
    if(items.size()<=0){
        id=type=resolution=fileSize="";
    }
    else {
        graphicsView->addImage(selectedImages[0].image);
        id=selectedImages[0].id.c_str();
        type=selectedImages[0].type;
        resolution=tr("%1 x %2").arg(selectedImages[0].cols()).arg(selectedImages[0].rows());
        fileSize=QString::number(selectedImages[0].fileSize())+" KB";
    }
    imageTable->setItem(0, 1, new QTableWidgetItem(id));
    imageTable->setItem(1, 1, new QTableWidgetItem(type));
    imageTable->setItem(2, 1, new QTableWidgetItem(resolution));
    imageTable->setItem(3, 1, new QTableWidgetItem(fileSize));

}

void ImageTree::updateScreen(QTreeWidgetItem * item, int)
{
    Qt::CheckState CheckState=item->checkState(0);
    std::vector<Index> indexs=this->getClickedIndexs(item);
    if(indexs.size()>1) {//parent
        for(int i=0;i<indexs.size();i++) {
            if(CheckState==Qt::Checked) {
                this->setImageChecked(indexs[i],true);
            }
            else {
                this->setImageChecked(indexs[i],false);
            }
        }
    } else {//child
        if(CheckState==Qt::Unchecked) {
            this->setImageChecked(indexs[0],false);
        }
        else {
            this->setImageChecked(indexs[0],true);
        }
    }
}




