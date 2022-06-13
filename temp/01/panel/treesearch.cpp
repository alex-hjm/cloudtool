#include "treesearch.h"
#include "ui_treesearch.h"

TreeSearch::TreeSearch(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::TreeSearch),search_cloud(new Cloud),is_picking(false)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&TreeSearch::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&TreeSearch::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&TreeSearch::reset);
    connect(ui->btn_pickpoint,&QPushButton::clicked,this,&TreeSearch::pick);
    connect(ui->cbox_type,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index) {
        if(index==0){
            ui->rbtn_resolution->hide();
            ui->dspin_resolution->hide();
        } else {
            ui->rbtn_resolution->show();
            ui->dspin_resolution->show();
        }
    });
    ui->rbtn_resolution->hide();
    ui->dspin_resolution->hide();

}

TreeSearch::~TreeSearch()
{
    delete ui;
}

void TreeSearch::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    cloud_tree->setExtendedSelection(false);
    connect(ui->spin_k,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value){
        if(!ui->rbtn_ksearch->isChecked())return;
        Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
        if(selectedCloud->empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if(ui->spin_point->value()>selectedCloud->size()) {
            console->warning(tr("Please set the correct parameters!"));
            return;
        }
        Cloud::Ptr searchPoint(new Cloud);
        searchPoint->push_back(selectedCloud->points[ui->spin_point->value()]);
        cloud_view->addCloud(searchPoint,"searchpoint");
        cloud_view->setCloudSize("searchpoint",5);
        cloud_view->setCloudColor(searchPoint,"searchpoint",255,255,255);
        if(ui->cbox_type->currentIndex()==0) {
            search_cloud=Tree::KDTree(selectedCloud,ui->spin_point->value(),value);
            cloud_view->addCloud(search_cloud,"kdtree");
            cloud_view->setCloudSize("kdtree",2);
            cloud_view->setCloudColor(search_cloud,"kdtree",255,0,0);
            cloud_view->showInfo("kdtree K-nearest neighbor search",30,"info");
        }
        else if(ui->cbox_type->currentIndex()==1) {
            search_cloud=Tree::OCTree(selectedCloud,ui->spin_point->value(),ui->dspin_resolution->value(),value);
            cloud_view->addCloud(search_cloud,"octree");
            cloud_view->setCloudSize("octree",2);
            cloud_view->setCloudColor(search_cloud,"octree",0,0,255);
            cloud_view->showInfo("octree K-nearest neighbor search",30,"info");
        }
    });

    connect(ui->dspin_radius,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->rbtn_rsearch->isChecked())return;
        Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
        if(selectedCloud->empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if(ui->spin_point->value()>selectedCloud->size()) {
            console->warning(tr("Please set the correct parameters!"));
            return;
        }
        Cloud::Ptr searchPoint(new Cloud);
        searchPoint->push_back(selectedCloud->points[ui->spin_point->value()]);
        cloud_view->addCloud(searchPoint,"searchpoint");
        cloud_view->setCloudSize("searchpoint",5);
        cloud_view->setCloudColor(searchPoint,"searchpoint",255,255,255);
        if(ui->cbox_type->currentIndex()==0) {
            search_cloud=Tree::KDTree(selectedCloud,ui->spin_point->value(),float(value));
            cloud_view->addCloud(search_cloud,"kdtree");
            cloud_view->setCloudSize("kdtree",2);
            cloud_view->setCloudColor(search_cloud,"kdtree",0,255,0);
            cloud_view->showInfo("kdtree R-radius search",30,"info");
        }
        else if(ui->cbox_type->currentIndex()==1) {
            search_cloud=Tree::OCTree(selectedCloud,ui->spin_point->value(),ui->dspin_resolution->value(),float(value));
            cloud_view->addCloud(search_cloud,"octree");
            cloud_view->setCloudSize("octree",2);
            cloud_view->setCloudColor(search_cloud,"octree",0,255,255);
            cloud_view->showInfo("octree R-radius search",30,"info");
        }
    });

    connect(ui->dspin_resolution,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->rbtn_resolution->isChecked())return;
        if(!ui->cbox_type->currentIndex()==1)return;
        Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
        if(selectedCloud->empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if(ui->spin_point->value()>selectedCloud->size()){
            console->warning(tr("Please set the correct parameters!"));
            return;
        }
        Cloud::Ptr searchPoint(new Cloud);
        searchPoint->push_back(selectedCloud->points[ui->spin_point->value()]);
        cloud_view->addCloud(searchPoint,"searchpoint");
        cloud_view->setCloudSize("searchpoint",5);
        cloud_view->setCloudColor(searchPoint,"searchpoint",255,255,255);
        search_cloud=Tree::OCTree(selectedCloud,ui->spin_point->value(),float(value));
        cloud_view->addCloud(search_cloud,"octree");
        cloud_view->setCloudSize("octree",2);
        cloud_view->setCloudColor(search_cloud,"octree",255,255,0);
        cloud_view->showInfo("octree voxel search",30,"info");
    });
}

void TreeSearch::pick()
{
    if(!is_picking){
        Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
        if(selectedCloud->empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        Index index=cloud_tree->getSelectedIndex();
        std::vector<Index> allIndexs=cloud_tree->getAllIndexs();
        for(auto &i:allIndexs)
            if(i!=index)
                cloud_tree->setCloudChecked(i,false);
        cloud_view->removeShape(selectedCloud->box_id);
        cloud_view->removeCloud("octree");
        cloud_view->removeCloud("kdtree");
        cloud_tree->setEnabled(false);
        is_picking=true;
        cloud_view->showInfo("Point Picking [ON] : Left click",30,"info");
        ui->btn_pickpoint->setText("stop");
        ui->btn_pickpoint->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        connect(cloud_view,&CloudView::mouseLeftPressed,this,&TreeSearch::pickPoint);
    }
    else {
        disconnect(cloud_view,&CloudView::mouseLeftPressed,this,&TreeSearch::pickPoint);
        cloud_view->removeShape("info");
        cloud_tree->setEnabled(true);
        is_picking=false;
        ui->btn_pickpoint->setText("pick");
        ui->btn_pickpoint->setIcon(QIcon(":/icon/resource/icon/start.svg"));
    }
}

void TreeSearch::add()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(search_cloud->empty()){
        console->warning(tr("Please search a pointcloud!"));
        return;
    }
    cloud_view->removeShape("info");
    cloud_view->removeCloud("octree");
    cloud_view->removeCloud("kdtree");
    cloud_view->removeCloud("searchpoint");
    Index index=cloud_tree->getSelectedIndex();
    search_cloud->copyInfo(selectedCloud);
    search_cloud->prefix("tree-");
    search_cloud->update();
    cloud_tree->setCloudChecked(index,false);
    cloud_tree->insertCloud(index.row,search_cloud,true);
    cloud_view->updateCube(search_cloud->box,search_cloud->box_id);
    console->info(tr("Added successfully!"));
    search_cloud->clear();
}

void TreeSearch::apply()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(search_cloud->empty()){
        console->warning(tr("Please search a pointcloud!"));
        return;
    }
    cloud_view->removeShape("info");
    cloud_view->removeCloud("octree");
    cloud_view->removeCloud("kdtree");
    cloud_view->removeCloud("searchpoint");
    selectedCloud->swap(*search_cloud);
    selectedCloud->update();
    search_cloud->clear();
    cloud_view->updateCloud(selectedCloud,selectedCloud->id);
    cloud_view->updateCube(selectedCloud->box,selectedCloud->box_id);
    console->info(tr("Applied successfully!"));
    search_cloud->clear();
}

void TreeSearch::reset()
{
    if(is_picking)
        this->pick();
    cloud_view->removeShape("info");
    cloud_view->removeCloud("octree");
    cloud_view->removeCloud("kdtree");
    cloud_view->removeCloud("searchpoint");
    search_cloud->clear();
}

void TreeSearch::pickPoint(const Point2D& pt)
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    int index=cloud_view->singlePick(pt);
    if(index==-1)return;
    ui->spin_point->setValue(index);
    Cloud::Ptr searchPoint(new Cloud);
    searchPoint->push_back(selectedCloud->points[index]);
    cloud_view->addCloud(searchPoint,"searchpoint");
    cloud_view->setCloudSize("searchpoint",5);
    cloud_view->setCloudColor(searchPoint,"searchpoint",255,255,255);
}

void TreeSearch::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloud_tree->setExtendedSelection(true);
    return QDockWidget::closeEvent(event);
}
