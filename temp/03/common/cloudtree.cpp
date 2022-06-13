#include "cloudtree.h"

CloudTree::CloudTree(QWidget *parent) :CustomTree(parent)
{
    FileIO *fileio=new FileIO;
    fileio->moveToThread(&thread);
    qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr &");
    qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr");
    connect(&thread,&QThread::finished,fileio,&QObject::deleteLater);
    connect(this,&CloudTree::loadPointCloud,fileio,&FileIO::loadPointCloud);
    connect(this,&CloudTree::savePointCloud,fileio,&FileIO::savePointCloud);
    connect(fileio,&FileIO::loadCloudResult,this,&CloudTree::loadCloudResult);
    connect(fileio,&FileIO::saveCloudResult,this,&CloudTree::saveCloudResult);
    thread.start();
    connect(this,&CloudTree::itemSelectionChanged,this,&CloudTree::updatePropertiesTable);
    connect(this,&CloudTree::itemClicked,this,&CloudTree::updateScreen);
}

CloudTree::~CloudTree()
{
    thread.quit();
    thread.wait();
}

void CloudTree::init(Console* &con,CloudView* &cv,QTableWidget* &qtw)
{
    console=con;cloud_view=cv;cloud_table=qtw;
}

void CloudTree::addClouds()
{

    QString filter ="all(*.*);;ply(*.ply);;pcd(*.pcd)";
    QStringList filePathList = QFileDialog::getOpenFileNames(this,tr("Open point cloud file"),"",filter);
    if (filePathList.isEmpty()) return ;
    console->showProgressBar();
    console->showStatusMessage("loading...",0);
    for (auto &i:filePathList)
        emit loadPointCloud(i);
}

void CloudTree::addSamlpe()
{
    QString path="../resource/clouds/rabbit.pcd";
    emit loadPointCloud(path);
}

bool CloudTree::insertCloud(int row,const Cloud::Ptr &cloud, bool selected)
{
    QIcon parent_icon(":/icon/resource/icon/document-open.svg");
    QIcon child_icon(":/icon/resource/icon/view-calendar.svg");
    if(this->containsId(cloud->id)) {
        QMessageBox message_box(QMessageBox::NoIcon,"",tr("Rename the exists id?"),QMessageBox::Yes|QMessageBox::Cancel,this,
                                Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        int k=message_box.exec();
        if(k==QMessageBox::Yes) {
            bool ok = false;
            QString res = QInputDialog::getText(this,"", tr("Rename"),QLineEdit::Normal, cloud->id.c_str(),&ok,
                                                Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
            if (ok){
                if(res==cloud->id.c_str()) {
                    console->warning(tr("The id ")+ res+tr(" already exists! Please rename a different id and retry."));
                    return false;
                } else {
                    if(cloud_view->contains(res.toStdString())) {
                        console->warning(tr("The id ")+ res+tr(" already exists! Please rename a different id and retry."));
                        return false;
                    }
                    cloud->rename(res.toStdString());
                    this->addItem<Cloud::Ptr>(row,cloud,cloud->path.c_str(),cloud->id.c_str(),parent_icon,child_icon,selected,cloud_vec);
                    cloud_view->addCloud(cloud,cloud->id);
                    return true;
                }
            } else {
                console->warning(tr("Add point cloud canceled."));
                return false;
            }
        } else{
            console->warning(tr("Add point cloud canceled."));
            return false;
        }
    } else {
        this->addItem<Cloud::Ptr>(row,cloud,cloud->path.c_str(),cloud->id.c_str(),parent_icon,child_icon,selected,cloud_vec);
        cloud_view->addCloud(cloud,cloud->id);
        return true;
    }
}

void CloudTree::clearCloud(const Index &index)
{
    if(index.row==-1||index.col==-1) return;
    Cloud::Ptr cloud=this->getItem<Cloud::Ptr>(index,cloud_vec);
    cloud->clear();
    cloud_view->removeCloud(cloud->id);
    cloud_view->removeShape(cloud->box_id);
    emit removedId(cloud->id);
    cloud_view->resetCamera();
    this->removeItem<Cloud::Ptr>(index,cloud_vec);
    for(auto &i:cloud_vec)
        std::vector<Cloud::Ptr>(i).swap(i);
    std::vector<std::vector<Cloud::Ptr>>(cloud_vec).swap(cloud_vec);
}

void CloudTree::clearSelectedClouds()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    this->getSortedIndexs(descending,indexs);
    for(auto &i:indexs)
        this->clearCloud(i);
}

void CloudTree::clearAllClouds()
{
    cloud_view->removeAllClouds();
    cloud_view->removeAllShapes();
    cloud_view->resetCamera();
    this->clear();
    this->cloud_vec.clear();
    for(auto &i:cloud_vec)
        std::vector<Cloud::Ptr>().swap(i);
    std::vector<std::vector<Cloud::Ptr>>().swap(cloud_vec);
}

void CloudTree::setCloudChecked(const Index &index, bool checked)
{
    if(index.row==-1||index.col==-1) return;
    this->setItemChecked(index,checked);
    Cloud::Ptr cloud=this->getItem<Cloud::Ptr>(index,cloud_vec);
    std::vector<Index> indexs=this->getSelectedIndexs();
    if(checked) {
        cloud_view->addCloud(cloud,cloud->id);
        for (auto &i:indexs)
            if(i==index)
                cloud_view->addCube(cloud->box,cloud->box_id);
    } else {
        cloud_view->removeCloud(cloud->id);
        cloud_view->removeShape(cloud->box_id);
    }
}

void CloudTree::setSelectedCloudsChecked(bool checked)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(auto &i:indexs)
        this->setCloudChecked(i,checked);
}

void CloudTree::setCloudSelected(const Index &index, bool selected)
{
    if(index.row==-1||index.col==-1) return;
    this->setItemSelected(index,selected);
    Cloud::Ptr cloud=this->getItem<Cloud::Ptr>(index,cloud_vec);
    if(selected)
        cloud_view->addCube(cloud->box,cloud->box_id);
    else
        cloud_view->removeShape(cloud->box_id);
}

void CloudTree::saveSelectedClouds()
{
    std::vector<Cloud::Ptr> selectedClouds=this->getSelectedClouds();
    for (auto &i:selectedClouds) {
        QString filter ="ply(*.ply);;pcd(*.pcd)";
        QString path= QFileDialog::getSaveFileName(this, tr ("Save point cloud"),"", filter);
        if (path.isEmpty ())
            continue;
        QMessageBox message_box(QMessageBox::NoIcon,"",tr("Save in binary or ascii format?"),QMessageBox::NoButton,this,
                                Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        message_box.addButton(tr("Ascii"), QMessageBox::ActionRole);
        message_box.addButton(tr("Binary"),QMessageBox::ActionRole)->setDefault(true);
        message_box.addButton(QMessageBox::Cancel);
        int k=message_box.exec();
        if(k==QMessageBox::Cancel){
            console->warning(tr("Save point cloud canceled."));
            continue;
        }
        emit savePointCloud(i,path,k);
    }
}

void CloudTree::mergeSelectedClouds()
{
    std::vector<Cloud::Ptr> selectedClouds=this->getSelectedClouds();
    if(selectedClouds.size()<=1) {
        console->warning(tr("The pointclouds to merge are not enough!"));
        return;
    }
    Cloud::Ptr merged_cloud(new Cloud);
    for(auto &i:selectedClouds)
        *merged_cloud+=*i;
    merged_cloud->copyInfo(selectedClouds.front());
    merged_cloud->rename("merged_cloud");
    merged_cloud->update();
    this->insertCloud(-1,merged_cloud,true);
}

void CloudTree::cloneSelectedClouds()
{
    std::vector<Cloud::Ptr> selectedClouds=this->getSelectedClouds();
    for(auto &i:selectedClouds) {
        Cloud::Ptr clone_cloud=i->makeShared();
        clone_cloud->prefix("clone-");
        this->insertCloud(-1,clone_cloud,true);
    }
}

void CloudTree::renameSelectedClouds()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(auto &i:indexs) {
        Cloud::Ptr cloud=this->getItem<Cloud::Ptr>(i,cloud_vec);
        bool ok = false;
        QString name = QInputDialog::getText(this,"", tr("Rename"),QLineEdit::Normal, cloud->id.c_str(),&ok,
                                             Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        if (ok) {
            this->renameItem(i,name);
            cloud_view->removeCloud(cloud->id);
            cloud_view->removeShape(cloud->box_id);
            cloud->rename(name.toStdString());
            cloud_view->addCloud(cloud,cloud->id);
            cloud_view->addCube(cloud->box,cloud->box_id);
        }
    }
}

Cloud::Ptr CloudTree::getCloud(const Index &index)
{
    return this->getItem<Cloud::Ptr>(index,cloud_vec);
}

Cloud::Ptr CloudTree::getSelectedCloud()
{
    Cloud::Ptr seletedCloud(new Cloud);
    if(getSelectedClouds().empty())return seletedCloud;
    else return getSelectedClouds().front();
}

std::vector<Cloud::Ptr> CloudTree::getSelectedClouds()
{
    std::vector<Index> indexs=getSelectedIndexs();
    std::vector<Cloud::Ptr> selectedClouds;
    for (auto &i:indexs)
        selectedClouds.push_back(this->getItem<Cloud::Ptr>(i,cloud_vec));
    return selectedClouds;
}

std::vector<Cloud::Ptr> CloudTree::getAllClouds()
{
    std::vector<Index> indexs=getAllIndexs();
    std::vector<Cloud::Ptr> allClouds;
    for (auto &i:indexs)
        allClouds.push_back(this->getItem<Cloud::Ptr>(i,cloud_vec));
    return allClouds;
}

void CloudTree::setAcceptDrops(bool enable)
{
    if(enable) {
        cloud_view->setAcceptDrops(true);
        connect(cloud_view,&CloudView::dropFilePath,[=](const QStringList &filepath) {
            console->showProgressBar();
            console->showStatusMessage("loading...",0);
            for (auto &i:filepath)
                emit loadPointCloud(i);
        });
    }
    else {
        cloud_view->setAcceptDrops(false);
        disconnect(cloud_view,&CloudView::dropFilePath,0,0);
    }
}

void CloudTree::setExtendedSelection(bool enable)
{
    if(enable)
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        this->setSelectionMode(QAbstractItemView::SingleSelection);
}

void CloudTree::loadCloudResult(bool success,const Cloud::Ptr &cloud,float time)
{
    if(!success)
        console->error(tr("Can not open the file!"));
    else {
        if(insertCloud(-1,cloud,false))
            console->info(tr("The pointcloud ")+cloud->id.c_str()+tr(" loaded successfully, take time :%1 ms.").arg(time));
    }
    console->clearStatusMessage();
    console->closeProgressBar();
    cloud_view->resetCamera();
}

void CloudTree::saveCloudResult(bool success,const QString& path,float time)
{
    if(!success)
        console->error(tr("The pointcloud save failed."));
    else
        console->info(tr("The pointcloud ")+ path +tr(" has been saved, take time %1 ms.").arg(time));
}

void CloudTree::updatePropertiesTable()
{
    std::vector<Cloud::Ptr> allClouds=this->getAllClouds();
    std::vector<Cloud::Ptr> selectedClouds=this->getSelectedClouds();
    for(auto &i:allClouds)
        cloud_view->removeShape(i->box_id);
    QString id,type,size,resolution;
    if(selectedClouds.empty()) {
        id=type=size=resolution="";
        cloud_table->removeCellWidget(4,1);
        cloud_table->removeCellWidget(5,1);
        cloud_view->showId("");
    } else {
        int total_size=0;
        for(auto &i:selectedClouds) {
            cloud_view->addCube(i->box,i->box_id);
            total_size+=i->size();
        }
        QSpinBox *point_size=new QSpinBox();
        point_size->setRange(1,99);
        QDoubleSpinBox *opacity=new QDoubleSpinBox();
        opacity->setSingleStep(0.1);
        opacity->setRange(0,1);
        cloud_table->setCellWidget(4,1,point_size);
        cloud_table->setCellWidget(5,1,opacity);
        if(selectedClouds.size()>1) {
            id="multi clouds";
            type="undefined";
            resolution="undefined";
            size=QString::number(total_size);
            point_size->setValue(1);
            opacity->setValue(1.0);
            cloud_view->showId("multi clouds");
        } else {
            id=QString::fromStdString(selectedClouds.front()->id);
            type=QString::fromStdString(selectedClouds.front()->type);
            resolution=QString::number(selectedClouds.front()->resolution);
            size=QString::number(selectedClouds.front()->size());
            point_size->setValue(selectedClouds.front()->point_size);
            opacity->setValue(selectedClouds.front()->opacity);
            cloud_view->showId(selectedClouds.front()->id);
        }
        connect(point_size,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value) {
            std::vector<Cloud::Ptr> selectedClouds=this->getSelectedClouds();
            for(auto &i:selectedClouds) {
                i->point_size=value;
                cloud_view->setCloudSize(i->id,value);
            }
        });
        connect(opacity,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
            std::vector<Cloud::Ptr> selectedClouds=this->getSelectedClouds();
            for(auto &i:selectedClouds) {
                i->opacity=value;
                cloud_view->setCloudOpacity(i->id,value);
            }
        });
    }
    cloud_table->setItem(0, 1, new QTableWidgetItem(id));
    cloud_table->setItem(1, 1, new QTableWidgetItem(type));
    cloud_table->setItem(2, 1, new QTableWidgetItem(size));
    cloud_table->setItem(3, 1, new QTableWidgetItem(resolution));
}

void CloudTree::updateScreen(QTreeWidgetItem * item, int)
{
    Qt::CheckState CheckState=item->checkState(0);
    std::vector<Index> indexs=this->getClickedIndexs(item);
    for(auto &i:indexs)
        if(CheckState==Qt::Checked)
            this->setCloudChecked(i,true);
        else
            this->setCloudChecked(i,false);
}

void CloudTree::mousePressEvent(QMouseEvent *event)
{
    QModelIndex indexSelect = indexAt(event->pos());
    if(indexSelect.row() == -1)
        setCurrentIndex(indexSelect);
    if(event->button() == Qt::RightButton && indexSelect.row() != -1){
        QTreeWidgetItem *item=this->itemFromIndex(indexSelect);
        if(item->isSelected()) {
            QMenu menu;
            QAction *clearAction = menu.addAction(QIcon(":/icon/resource/icon/close-copy.svg"),"clear");
            connect(clearAction,&QAction::triggered,this,&CloudTree::clearSelectedClouds);
            QAction *saveAction = menu.addAction(QIcon(":/icon/resource/icon/document-save.svg"),"save");
            connect(saveAction,&QAction::triggered,this,&CloudTree::saveSelectedClouds);
            if(item->checkState(0)==Qt::Checked) {
                QAction *hideAction = menu.addAction(QIcon(":/icon/resource/icon/view-hidden.svg"),"hide");
                connect(hideAction,&QAction::triggered,this,[=]{this->setSelectedCloudsChecked(false);});
            } else if(item->checkState(0)==Qt::Unchecked||item->checkState(0)==Qt::PartiallyChecked) {
                QAction *showAction = menu.addAction(QIcon(":/icon/resource/icon/view-show.svg"),"show");
                connect(showAction,&QAction::triggered,this,[=]{this->setSelectedCloudsChecked(true);});
            }
            QAction *cloneAction = menu.addAction(QIcon(":/icon/resource/icon/split.svg"),"clone");
            connect(cloneAction,&QAction::triggered,this,&CloudTree::cloneSelectedClouds);
            QAction *renameAction = menu.addAction(QIcon(":/icon/resource/icon/amarok_scripts.svg"),"rename");
            connect(renameAction,&QAction::triggered,this,&CloudTree::renameSelectedClouds);
            menu.exec(event->globalPos());
            event->accept();
        }
    }
    return CustomTree::mousePressEvent(event);
}
