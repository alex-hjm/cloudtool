#include "imagetree.h"

ImageTree::ImageTree(QWidget *parent):
    CustomTree(parent)
{
    connect(this,&ImageTree::itemSelectionChanged,this,&ImageTree::updatePropertiesTable);
    connect(this,&ImageTree::itemClicked,this,&ImageTree::updateScreen);
}

void ImageTree::init(Console* &con,ImageView* &iv,QTableWidget* &qtw)
{
    console=con;image_view=iv;image_table=qtw;
}

void ImageTree::addImages()
{
    QString filter ="all(*.*);;jpg(*.jpg);;jpg(*.png)";
    QStringList filePathList = QFileDialog::getOpenFileNames(this,tr("Open image file"),"",filter);
    if (filePathList.isEmpty()) return;
    console->showStatusMessage("loading...",0);
    int num=0;
    for (auto &i:filePathList) {
        Image::Ptr image(new Image);
        if(!FileIO::loadImage(image,i))
            console->error(tr("Can not open the file :")+i);
        else {
            insertImage(num,image,false);
            num++;
            cv::waitKey(10);
        }
    }
    console->info(tr("Load %1 images successfully!").arg(num));
    console->clearStatusMessage();
}

void ImageTree::insertImage(int num, const Image::Ptr &image,const bool&selected)
{
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/child2category.svg");
    int currentindex=this->topLevelItemCount()-1;
    if(num==0){
        this->addItem<Image::Ptr>(-1,image,image->path(),image->id.c_str(),parentIcon,childIcon,selected,image_vec);
        image_view->addImage(image,true);
    }
    else{
        this->addItem<Image::Ptr>(currentindex,image,image->path(),image->id.c_str(),parentIcon,childIcon,selected,image_vec);
        image_view->addImage(image,true);
    }
}

void ImageTree::clearImage(const Index &index)
{
    if(index.row==-1||index.col==-1) return;
    this->removeItem<Image::Ptr>(index,image_vec);
    image_view->removeImage();
    for(auto &i:image_vec)
        std::vector<Image::Ptr>(i).swap(i);
    std::vector<std::vector<Image::Ptr>>(image_vec).swap(image_vec);
}

void ImageTree::clearSelectedImages()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    this->getSortedIndexs(descending,indexs);
    for(auto &i:indexs)
        this->clearImage(i);
}

void ImageTree::clearAllImages()
{
    image_view->removeImage();
    this->clear();
    this->image_vec.clear();
    for(auto &i:image_vec)
        std::vector<Image::Ptr>().swap(i);
    std::vector<std::vector<Image::Ptr>>().swap(image_vec);
}

void ImageTree::setImageChecked(const Index &index, bool checked)
{
    if(index.row==-1||index.col==-1) return;
    this->setItemChecked(index,checked);
    Image::Ptr image=this->getItem<Image::Ptr>(index,image_vec);
    std::vector<Index> indexs=this->getSelectedIndexs();
    if(indexs.empty())return;
    if(indexs.back()==index)
        if(checked)
            image_view->addImage(image,true);
        else
            image_view->removeImage();
}

void ImageTree::setSelectedImagesChecked(bool checked)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(auto &i:indexs)
        this->setImageChecked(i,checked);
}

void ImageTree::setImageSelected(const Index &index, bool selected)
{
    if(index.row==-1||index.col==-1) return;
    this->setItemSelected(index,selected);
    if(!selected)
        image_view->removeImage();
}


void ImageTree::saveSelectedImages()
{
    std::vector<Image::Ptr> selectedImages=this->getSelectedImages();
    for (auto &i:selectedImages) {
        QString filter ="jpg(*.jpg);;png(*.png)";
        QString path = QFileDialog::getSaveFileName(this, tr ("Save image"),"", filter);
        if (path.isEmpty ())
            continue;
        if(!FileIO::saveImage(i,path)) {
            console->error(tr("The image save failed."));
            return;
        }
        console->info(tr("The image ")+ path+tr(" has been saved."));
    }
}

void ImageTree::cloneSelectedImages()
{
    std::vector<Image::Ptr> selectedImages=this->getSelectedImages();
    for(auto &i:selectedImages) {
        Image::Ptr clone_image=i->makeShared();
        clone_image->prefix("clone-");
        this->insertImage(-1,clone_image,true);
    }
}

void ImageTree::renameSelectedImages()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(auto &i:indexs) {
        Image::Ptr cloud=this->getItem<Image::Ptr>(i,image_vec);
        bool ok = false;
        QString name = QInputDialog::getText(this,"", tr("Rename"),QLineEdit::Normal, cloud->id.c_str(),&ok,
                                             Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        if (ok) {
            this->renameItem(i,name);
            cloud->rename(name.toStdString());
        }
    }
}

Image::Ptr ImageTree::getSelectedImage()
{
    Image::Ptr seletedImage(new Image);
    if(getSelectedImages().empty())return seletedImage;
    else return getSelectedImages().front();
}

std::vector<Image::Ptr> ImageTree::getCheckedImages()
{
    std::vector<Index> indexs=getCheckedIndexs();
    std::vector<Image::Ptr> checkedImages;
    for (auto &i:indexs)
        checkedImages.push_back(this->getItem<Image::Ptr>(i,image_vec));
    return checkedImages;
}

std::vector<Image::Ptr> ImageTree::getSelectedImages()
{
    std::vector<Index> indexs=getSelectedIndexs();
    std::vector<Image::Ptr> selectedImages;
    for (auto &i:indexs)
        selectedImages.push_back(this->getItem<Image::Ptr>(i,image_vec));
    return selectedImages;
}

std::vector<Image::Ptr> ImageTree::getAllImages()
{
    std::vector<Index> indexs=getAllIndexs();
    std::vector<Image::Ptr> allImages;
    for (auto &i:indexs)
        allImages.push_back(this->getItem<Image::Ptr>(i,image_vec));
    return allImages;
}

void ImageTree::setAcceptDrops(const bool &enable)
{
    if(enable) {
        image_view->setAcceptDrops(true);
        connect(image_view,&ImageView::dropFilePath,[=](QStringList path) {
            if (path.isEmpty()) return;
            console->showStatusMessage("loading...",0);
            int num=0;
            for (auto &i:path) {
                Image::Ptr image(new Image);
                if(!FileIO::loadImage(image,i))
                    console->error(tr("Can not open the file :")+i);
                else {
                    insertImage(num,image,false);
                    num++;
                    cv::waitKey(10);
                }
            }
            console->info(tr("Load %1 images successfully!").arg(num));
            console->clearStatusMessage();
        });
    }
    else {
        image_view->setAcceptDrops(false);
        disconnect(image_view,&ImageView::dropFilePath,0,0);
    }
}

void ImageTree::setExtendedSelection(const bool &enable)
{
    if(enable)
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        this->setSelectionMode(QAbstractItemView::SingleSelection);
}


void ImageTree::updatePropertiesTable()
{
    std::vector<Image::Ptr> selectedImages=this->getSelectedImages();
    QString id,type,file_size,resolution,birth_time;
    if(selectedImages.empty()) {
        image_view->removeImage();
        id=type=file_size=resolution=birth_time="";
    }
    else {
        if(selectedImages.size()>1)
            image_view->addImage(selectedImages.back(),true);
        id=selectedImages.back()->id.c_str();
        type=selectedImages.back()->type();
        file_size=tr("%1 KB").arg(selectedImages.back()->fileSize());
        resolution=tr("%1 x %2").arg(selectedImages.back()->cols).arg(selectedImages.back()->rows);
        birth_time=selectedImages.back()->birthTime().toString("yyyy-MM-dd");
    }
    image_table->setItem(0, 1, new QTableWidgetItem(id));
    image_table->setItem(1, 1, new QTableWidgetItem(type));
    image_table->setItem(2, 1, new QTableWidgetItem(file_size));
    image_table->setItem(3, 1, new QTableWidgetItem(resolution));
    image_table->setItem(4, 1, new QTableWidgetItem(birth_time));
}

void ImageTree::updateScreen(QTreeWidgetItem * item, int)
{
    Qt::CheckState CheckState=item->checkState(0);
    std::vector<Index> indexs=this->getClickedIndexs(item);
    if(indexs.size()>1) {
        for(auto &i:indexs)
            if(CheckState==Qt::Checked)
                this->setImageChecked(i,true);
            else
                this->setImageChecked(i,false);
    }else {
            if(CheckState==Qt::Unchecked) {
                this->setImageChecked(indexs.front(),false);
            }
            else {
                this->setImageChecked(indexs.front(),true);
            }
        }
}

void ImageTree::mousePressEvent(QMouseEvent *event)
{
    QModelIndex indexSelect = indexAt(event->pos());
    if(indexSelect.row() == -1)
        setCurrentIndex(indexSelect);
    if(event->button() == Qt::RightButton && indexSelect.row() != -1){
        QTreeWidgetItem *item=this->itemFromIndex(indexSelect);
        if(item->isSelected()) {
            QMenu menu;
            QAction *clearAction = menu.addAction(QIcon(":/icon/resource/icon/document-close.svg"),"clear");
            connect(clearAction,&QAction::triggered,this,&ImageTree::clearSelectedImages);
            QAction *saveAction = menu.addAction(QIcon(":/icon/resource/icon/document-save.svg"),"save");
            connect(saveAction,&QAction::triggered,this,&ImageTree::saveSelectedImages);
            if(item->checkState(0)==Qt::Checked) {
                QAction *hideAction = menu.addAction(QIcon(":/icon/resource/icon/view-hidden.svg"),"hide");
                connect(hideAction,&QAction::triggered,this,[=]{this->setSelectedImagesChecked(false);});
            } else if(item->checkState(0)==Qt::Unchecked) {
                QAction *showAction = menu.addAction(QIcon(":/icon/resource/icon/view-show.svg"),"show");
                connect(showAction,&QAction::triggered,this,[=]{this->setSelectedImagesChecked(true);});
            }
            QAction *cloneAction = menu.addAction(QIcon(":/icon/resource/icon/split.svg"),"clone");
            connect(cloneAction,&QAction::triggered,this,&ImageTree::cloneSelectedImages);
            QAction *renameAction = menu.addAction(QIcon(":/icon/resource/icon/amarok_scripts.svg"),"rename");
            connect(renameAction,&QAction::triggered,this,&ImageTree::renameSelectedImages);
            menu.exec(event->globalPos());
            event->accept();
        }
    }
    return CustomTree::mousePressEvent(event);
}



