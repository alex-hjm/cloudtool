#include "cloudtree.h"

CloudTree::CloudTree(QWidget *parent) :
    CustomTree(parent),
    lastFileInfo(QFileInfo("../resource/clouds/"))
{
    connect(this,&CloudTree::itemSelectionChanged,this,&CloudTree::updateProperty);
    connect(this,&CloudTree::itemClicked,this,&CloudTree::updateScreen);
}

bool CloudTree::loadCloud(const QString &path)
{
    lastFileInfo.setFile(path);
    CloudXYZRGBN::Ptr m_cloud(new CloudXYZRGBN);
    if(!fileio.loadPointCloud(m_cloud,lastFileInfo)) {
        console->error(tr("Can not open the file!"));
        return false;
    }
    Cloud cloud(m_cloud,lastFileInfo);
    console->info(tr("The pointcloud ")+lastFileInfo.fileName()+tr(" loaded successfully! File size :%1 KB ,Take time :%2 ms.").arg(cloud.fileSize()).arg(fileio.tocTime));
    this->insertCloud(-1,cloud,false);
    return true;
}


void CloudTree::addClouds()
{
    QString filter ="all(*.*);;ply(*.ply);;pcd(*.pcd);;ifs(*.ifs);;obj(*.obj)";
    QStringList filePathList = QFileDialog::getOpenFileNames(this,tr("Open point cloud file"),lastFileInfo.filePath(),filter);
    if (filePathList.isEmpty()) return ;
    console->info(tr("Loading..."));
    for (int i = 0; i <filePathList.size(); i++) {
        if(!loadCloud(filePathList[i]))continue;
    }
    cloudView->resetCamera();
}

void CloudTree::addSamlpeCloud()
{
    this->loadCloud("../resource/clouds/rabbit.pcd");
    cloudView->resetCamera();
}

void CloudTree::insertCloud(int column,Cloud &cloud,const bool&selected)
{
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/view-calendar.svg");
    if(cloudView->contains(cloud.id)) {
        int k= QMessageBox::information(this,"",tr("The same name exists"),tr("rename"),tr("cancel"));
        if(k==0) {
            bool ok = false;
            QString res = QInputDialog::getText(this, tr("Rename"), tr("Rename"),
                                                QLineEdit::Normal, cloud.id.c_str(),&ok, Qt::WindowFlags());
            if (ok) {
                if(res==cloud.id.c_str()) {
                    console->warning(tr("The id ")+ res+tr(" already exists! Please rename a different id and retry."));
                    return ;
                }
                else {
                    if(cloudView->contains(res.toStdString())) {
                        console->warning(tr("The id ")+ res+tr(" already exists! Please rename a different id and retry."));
                        return ;
                    }
                    cloud.rename(res.toStdString());
                    this->addItem<Cloud>(column,cloud,cloud.path(),cloud.id.c_str(),parentIcon,childIcon,selected,cloudVec);
                    cloudView->updateCloud(cloud.cloud,cloud.id);
                }
            }
        } else{
            console->warning(tr("Add point cloud canceled."));
            return ;
        }
    }
    else {
        this->addItem<Cloud>(column,cloud,cloud.path(),cloud.id.c_str(),parentIcon,childIcon,selected,cloudVec);
        cloudView->updateCloud(cloud.cloud,cloud.id);
    }
}

void CloudTree::updateCloud(const Index &index,Cloud &updatecloud)
{
    updatecloud.update();
    this->updateItem<Cloud>(index,updatecloud,cloudVec);
    cloudView->updateCloud(updatecloud.cloud,updatecloud.id);
}

void CloudTree::updateSelectedClouds(std::vector<Cloud> &updateclouds)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->updateCloud(indexs[i],updateclouds[i]);
    }
}

void CloudTree::clearCloud(const Index &index)
{
    Cloud cloud=this->getCloud(index);
    cloudView->removeCloud(cloud.id);
    cloudView->removeCloud(cloud.normalsid);
    cloudView->removeShape(cloud.boxid);
    this->removeItem<Cloud>(index,cloudVec);
    emit removedId(cloud.id);
    if(this->size()<=0)
        cloudView->showCloudId("CloudTool");
    else
        cloudView->showCloudId("");
    cloudView->resetCamera();
}

void CloudTree::clearSelectedClouds()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    indexs=this->getSortedIndexs(descending,indexs);
    for(int i=0;i<indexs.size();i++) {
        this->clearCloud(indexs[i]);
    }
}

void CloudTree::clearAllClouds()
{
    this->cloudVec.clear();
    this->clear();
    cloudView->removeAllClouds();
    cloudView->removeAllShapes();
    cloudView->showCloudId("CloudTool");
    cloudView->resetCamera();
}

void CloudTree::setCloudChecked(const Index &index, bool checked)
{
    if(index.row==-1) return;
    std::vector<Index> indexs=this->getSelectedIndexs();
    this->setItemChecked(index,checked);
    Cloud cloud=this->getCloud(index);
    if(checked) {
        cloudView->updateCloud(cloud.cloud,cloud.id);
        for(int i=0;i<indexs.size();i++)
            if(indexs[i]==index)
                cloudView->updateBoundingBox(cloud.box,cloud.boxid);
    }
    else {
        cloudView->removeCloud(cloud.id);
        cloudView->removeShape(cloud.boxid);
    }
}

void CloudTree::setSelectedCloudsChecked(bool checked)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->setCloudChecked(indexs[i],checked);
    }
}

void CloudTree::setCloudSelected(const Index &index, bool selected)
{
    this->setItemSelected(index,selected);
    Cloud cloud=this->getCloud(index);
    if(selected) {
        this->setCloudChecked(index,true);
        cloudView->updateBoundingBox(cloud.box,cloud.boxid);
    }
    else
        cloudView->removeShape(cloud.boxid);
}

void CloudTree::setSelectedCloudsSelected(bool selected)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->setCloudSelected(indexs[i],selected);
    }
}

void CloudTree::saveSelectedClouds()
{
    std::vector<Cloud> saveClouds;
    std::vector<Cloud> selectedClouds=this->getSelectedClouds();
    if(selectedClouds.size()>1) {
        int k= QMessageBox::information(this,"Tips",tr("Merge or not?"),tr("yes"),tr("cancel"));
        if(k==0) {
            saveClouds.push_back(this->mergeClouds(selectedClouds));
        }
        else {
            saveClouds=selectedClouds;
        }
    }
    else if(selectedClouds.size()==1) {
        saveClouds=selectedClouds;
    }
    else {
        console->warning(tr("Please select the pointclouds."));
        return ;
    }
    for (int i=0;i<saveClouds.size();i++) {
        QString filter ="ply(*.ply);;pcd(*.pcd);;ifs(*.ifs);;obj(*.obj)";
        lastFileInfo = QFileDialog::getSaveFileName(this, tr ("Save point cloud"),lastFileInfo.filePath(), filter);
        if (lastFileInfo.filePath().isEmpty ())
            continue;
        int k=QMessageBox::information(this,tr("choose output format"),tr("Save in Binary or Ascii format?"),tr("Ascii"),tr("Binary"),tr("Cancel"));
        if(k==2)continue;
        if(!fileio.savePointCloud(saveClouds[i].cloud,lastFileInfo,k)) {
            console->error(tr("The pointcloud save failed."));
            return;
        }
        console->info(tr("The pointcloud ")+ lastFileInfo.fileName()+tr(" has been saved,take time %1 ms.").arg(fileio.tocTime));
    }
}

void CloudTree::mergeSelectedClouds()
{
    std::vector<Cloud> selectedClouds=this->getSelectedClouds();
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/view-calendar.svg");
    if(selectedClouds.size()<=1) {
        console->warning(tr("The pointclouds to merge are not enough!"));
        return;
    }
    Cloud merge_cloud=mergeClouds(selectedClouds);
    this->addItem<Cloud>(-1,merge_cloud,merge_cloud.path(),merge_cloud.id.c_str(),parentIcon,childIcon,false,cloudVec);
    cloudView->updateCloud(merge_cloud.cloud,merge_cloud.id);
}

void CloudTree::cloneSelectedClouds()
{
    std::vector<Cloud> selectedClouds=this->getSelectedClouds();
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/view-calendar.svg");
    for(size_t i=0;i<selectedClouds.size();i++) {
        CloudXYZRGBN::Ptr c_cloud (new CloudXYZRGBN);
        *c_cloud=*selectedClouds[i].cloud;
        Cloud clone_cloud(c_cloud,selectedClouds[i].fileInfo);
        clone_cloud.prefix("clone-");
        this->addItem<Cloud>(-1,clone_cloud,clone_cloud.path(),clone_cloud.id.c_str(),parentIcon,childIcon,false,cloudVec);
        cloudView->updateCloud(clone_cloud.cloud,clone_cloud.id);
    }
}

void CloudTree::renameSelectedClouds()
{
    std::vector<Cloud> selectedClouds=this->getSelectedClouds();
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        bool ok = false;
        QString name = QInputDialog::getText(this, tr("Rename"), tr("Rename"),
                                             QLineEdit::Normal, selectedClouds[i].id.c_str(),&ok, Qt::WindowFlags());
        if (ok) {
            this->renameItem(indexs[i],name);
            cloudView->removeCloud(selectedClouds[i].id);
            cloudView->removeCloud(selectedClouds[i].normalsid);
            cloudView->removeShape(selectedClouds[i].boxid);
            selectedClouds[i].rename(name.toStdString());
            this->updateCloud(indexs[i],selectedClouds[i]);
            cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
        }
    }
}

Cloud CloudTree::mergeClouds(std::vector<Cloud> &clouds)
{
    CloudXYZRGBN::Ptr m_cloud (new CloudXYZRGBN);
    std::string id;
    for (int i=0;i<clouds.size();i++) {
        *m_cloud+=*(clouds[i].cloud);
        id+="-"+clouds[i].id;
    }
    Cloud merge_cloud(m_cloud,clouds[0].fileInfo);
    merge_cloud.rename("merge"+id);
    return merge_cloud;
}

Cloud CloudTree::getCloud(const Index &index)
{
    return this->getData<Cloud>(index,cloudVec);
}

Cloud CloudTree::getSelectedCloud()
{
    Cloud selectedCloud;
    if(this->getSelectedClouds().size()<=0)
        return selectedCloud;
    else
        return this->getSelectedClouds()[0];
}

std::vector<Cloud> CloudTree::getSelectedClouds()
{

    return this->getSelectedDatas<Cloud>(cloudVec);
}

std::vector<Cloud> CloudTree::getAllClouds()
{
    return this->getAllDatas<Cloud>(cloudVec);
}


void CloudTree::setAcceptDrops(const bool &enable)
{
    if(enable) {
        cloudView->setAcceptDrops(true);
        connect(cloudView,&CloudView::dropFilePath,[=](QStringList filepath) {
            for (int i = 0; i <filepath.size(); i++) {
                if(!loadCloud(filepath[i]))continue;
            }
            cloudView->resetCamera();
        });
    }
    else {
        cloudView->setAcceptDrops(false);
        disconnect(cloudView,&CloudView::dropFilePath,0,0);
    }
}

void CloudTree::setExtendedSelection(const bool &enable)
{
    if(enable)
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        this->setSelectionMode(QAbstractItemView::SingleSelection);

}

void CloudTree::updateProperty()
{
    std::vector<Cloud> allClouds=this->getAllClouds();
    QString id,type,size,resolution;
    auto items = this->selectedItems();
    if(items.size()<=0) {
        for(size_t i=0;i<allClouds.size();i++) {
            cloudView->removeShape(allClouds[i].boxid);
            cloudView->removeShape(allClouds[i].normalsid);
        }
        id=type=size=resolution="";
        cloudTable->removeCellWidget(4,1);
        cloudTable->removeCellWidget(5,1);
        cloudTable->removeCellWidget(6,1);
        cloudView->showCloudId("");
    }
    else {
        std::vector<Cloud> selectedClouds=this->getSelectedClouds();
        for(size_t i=0;i<allClouds.size();i++)
            cloudView->removeShape(allClouds[i].boxid);
        for(size_t i=0;i<selectedClouds.size();i++)
            cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
        //update
        if(selectedClouds.size()>1) {
            id="multi clouds";
            type="default";
            resolution="default";
            int total_size=0;
            for(size_t i=0;i<selectedClouds.size();i++) {
                total_size+=selectedClouds[i].size();
            }
            size=QString::number(total_size);
            cloudView->showCloudId("multi clouds");
        } else {
            id=QString::fromStdString(selectedClouds[0].id);
            type=selectedClouds[0].type;
            resolution=QString::number(selectedClouds[0].resolution);
            size=QString::number(selectedClouds[0].size());
            cloudView->showCloudId(selectedClouds[0].id);
        }
        //pointsize
        QSpinBox *point_size=new QSpinBox();
        point_size->setMaximumHeight(24);
        point_size->setRange(1,99);
        if(selectedClouds.size()>1)
            point_size->setValue(1);
        else
            point_size->setValue(selectedClouds[0].pointSize);
        connect(point_size,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value) {
            std::vector<Cloud> selectedClouds=this->getSelectedClouds();
            std::vector<Index> indexs=this->getSelectedIndexs();
            for(size_t i=0;i<selectedClouds.size();i++) {
                cloudView->setCloudSize(selectedClouds[i].id,value);
                selectedClouds[i].pointSize=value;
                this->updateCloud(indexs[i],selectedClouds[i]);
            }
        });
        cloudTable->setCellWidget(4,1,point_size);
        //opacity
        QDoubleSpinBox *Opacity=new QDoubleSpinBox();
        Opacity->setMaximumHeight(24);
        Opacity->setSingleStep(0.1);
        Opacity->setRange(0,1);
        if(selectedClouds.size()>1)
            Opacity->setValue(1.0);
        else
            Opacity->setValue(selectedClouds[0].opacity);
        connect(Opacity,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
            std::vector<Cloud> selectedClouds=this->getSelectedClouds();
            std::vector<Index> indexs=this->getSelectedIndexs();
            for(size_t i=0;i<selectedClouds.size();i++) {
                cloudView->setCloudOpacity(selectedClouds[i].id,value);
                selectedClouds[i].opacity=value;
                this->updateCloud(indexs[i],selectedClouds[i]);
            }
        });
        cloudTable->setCellWidget(5,1,Opacity);
        //normal
        QCheckBox *showNormals=new QCheckBox();
        showNormals->setMaximumHeight(24);
        if(!selectedClouds[0].hasNormals|(selectedClouds.size()>1))
            showNormals->setEnabled(false);
        connect(showNormals,&QCheckBox::stateChanged,[=](int state) {
            std::vector<Cloud> selectedClouds=this->getSelectedClouds();
            if(state)
                cloudView->updateNormols(selectedClouds[0].cloud,1,0.01,selectedClouds[0].normalsid);
            else
                cloudView->removeShape(selectedClouds[0].normalsid);
        });
        cloudTable->setCellWidget(6,1,showNormals);
    }
    cloudTable->setItem(0, 1, new QTableWidgetItem(id));
    cloudTable->setItem(1, 1, new QTableWidgetItem(type));
    cloudTable->setItem(2, 1, new QTableWidgetItem(size));
    cloudTable->setItem(3, 1, new QTableWidgetItem(resolution));
}

void CloudTree::updateScreen(QTreeWidgetItem * item, int)
{
    Qt::CheckState CheckState=item->checkState(0);
    std::vector<Index> indexs=this->getClickedIndexs(item);
    if(indexs.size()>1) {//parent
        for(int i=0;i<indexs.size();i++) {
            if(CheckState==Qt::Checked) {
                this->setCloudChecked(indexs[i],true);
            }
            else {
                this->setCloudChecked(indexs[i],false);
            }
        }
    } else {//child
        if(CheckState==Qt::Unchecked) {
            this->setCloudChecked(indexs[0],false);
        }
        else {
            this->setCloudChecked(indexs[0],true);
        }
    }
}

void CloudTree::mousePressEvent(QMouseEvent *event)
{
    QModelIndex indexSelect = indexAt(event->pos());
    if(indexSelect.row() == -1)
        setCurrentIndex(indexSelect);
    if(event->button() == Qt::RightButton)
    {
        if(indexSelect.row() != -1) {
            QTreeWidgetItem *item=this->itemFromIndex(indexSelect);
            if(item->isSelected()) {
                QMenu menu;
                QAction *clearAction = menu.addAction("clear");
                connect(clearAction,&QAction::triggered,this,&CloudTree::clearSelectedClouds);
                QAction *saveAction = menu.addAction("save");
                connect(saveAction,&QAction::triggered,this,&CloudTree::saveSelectedClouds);
                if(item->checkState(0)==Qt::Checked) {
                    QAction *hideAction = menu.addAction("hide");
                    connect(hideAction,&QAction::triggered,this,[=]{this->setSelectedCloudsChecked(false);});
                } else if(item->checkState(0)==Qt::Unchecked) {
                    QAction *showAction = menu.addAction("show");
                    connect(showAction,&QAction::triggered,this,[=]{this->setSelectedCloudsChecked(true);});
                }
                QAction *cloneAction = menu.addAction("clone");
                connect(cloneAction,&QAction::triggered,this,&CloudTree::cloneSelectedClouds);
                QAction *renameAction = menu.addAction("rename");
                connect(renameAction,&QAction::triggered,this,&CloudTree::renameSelectedClouds);
                QAction *normalsAction = menu.addAction("normals");
                connect(normalsAction,&QAction::triggered,this,[=]{
                    std::vector<Cloud> selectedClouds=this->getSelectedClouds();
                    for(int i=0;i<selectedClouds.size();i++) {
                        if(!selectedClouds[i].hasNormals) return;
                        if(cloudView->contains(selectedClouds[i].normalsid))
                            cloudView->removeShape(selectedClouds[i].normalsid);
                        else
                            cloudView->updateNormols(selectedClouds[i].cloud,1,0.01,selectedClouds[i].normalsid);
                    }
                });
                QAction *keypointsAction = menu.addAction("keypoints");
                connect(keypointsAction,&QAction::triggered,this,[=]{
                    std::vector<Cloud> selectedClouds=this->getSelectedClouds();
                    for(int i=0;i<selectedClouds.size();i++) {
                        if(selectedClouds[i].keypoints->empty()) return;
                        if(cloudView->contains(selectedClouds[i].keyid))
                            cloudView->removeShape(selectedClouds[i].keyid);
                        else {
                            cloudView->updateCloud(selectedClouds[i].keypoints,selectedClouds[i].keyid);
                            cloudView->setCloudColor(selectedClouds[i].keypoints,selectedClouds[i].keyid,0,0,255);
                            cloudView->setCloudSize(selectedClouds[i].keyid,5);
                        }
                    }
                });
                menu.exec(event->globalPos());
                event->accept();
            }
        }
    }
    return QTreeWidget::mousePressEvent(event);
}
