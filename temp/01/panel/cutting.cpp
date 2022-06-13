#include "cutting.h"
#include "ui_cutting.h"

Cutting::Cutting(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Cutting),cutted_cloud(new Cloud),pick_type(0),is_picking(false),pick_start(false)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Cutting::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Cutting::apply);
    connect(ui->btn_start,&QPushButton::clicked,this,&Cutting::start);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Cutting::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Cutting::close);
    connect(ui->btn_selectin,&QPushButton::clicked,[=]{this->cuttingCloud(false);});
    connect(ui->btn_selectout,&QPushButton::clicked,[=]{this->cuttingCloud(true);});
    ui->cbox_type->setCurrentIndex(0);
}

Cutting::~Cutting()
{
    delete ui;
}

void Cutting::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    cloud_tree->setExtendedSelection(false);
    connect(ui->cbox_type,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index) {
        pick_type=index;
        if(index==0) {
            if(is_picking)
                cloud_view->showInfo("Segmentation [ON] (rectangular selection)",30,"info");
            else
                cloud_view->showInfo("Segmentation [OFF] (rectangular selection)",30,"info");
            cloud_view->showInfo("Left/Right click : set opposite corners",50,"info1");
        } else {
            if(is_picking)
                cloud_view->showInfo("Segmentation [ON] (polygonal selection)",30,"info");
            else
                cloud_view->showInfo("Segmentation [OFF] (polygonal selection)",30,"info");
            cloud_view->showInfo("Left click : add contour points / Right click : close",50,"info1");
        }
    });
    cloud_view->showInfo("Cuttingation [OFF] (rectangular selection)",30,"info");
    cloud_view->showInfo("Left/Right click : set opposite corners",50,"info1");

}

void Cutting::add()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(cutted_cloud->empty()) {
        console->warning(tr("Please Cutting a pointcloud!"));
        return;
    }
    Index index=cloud_tree->getSelectedIndex();
    cloud_view->removeCloud("cut");
    cloud_view->removeShape("cut-box");
    cutted_cloud->prefix("cutted-");
    cutted_cloud->update();
    cloud_tree->insertCloud(index.row,cutted_cloud,true);
    cloud_view->updateCube(cutted_cloud->box,cutted_cloud->box_id);
    console->info(tr("Added successfully!"));
    cutted_cloud->clear();
}

void Cutting::apply()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(cutted_cloud->empty()) {
        console->warning(tr("Please Cutting a pointcloud!"));
        return;
    }
    Index index=cloud_tree->getSelectedIndex();
    cloud_view->removeCloud("cut");
    cloud_view->removeShape("cut-box");
    selectedCloud->swap(*cutted_cloud);
    selectedCloud->update();
    cutted_cloud->clear();
    cloud_tree->setCloudChecked(index,true);
    cloud_view->updateCube(selectedCloud->box,selectedCloud->box_id);
    console->info(tr("Applied successfully!"));
    cutted_cloud->clear();
}

void Cutting::start()
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
            else
                cloud_tree->setCloudChecked(i,true);
        cloud_view->removeShape(selectedCloud->box_id);
        cloud_view->setInteractorEnable(false);
        cloud_tree->setEnabled(false);
        is_picking=true;
        pick_start=false;
        cloud_view->removeShape("poly");
        cloud_view->removeShape("cut-box");
        points.clear();
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        //connect
        connect(cloud_view,&CloudView::mouseLeftPressed,this,&Cutting::leftPressPoint);
        connect(cloud_view,&CloudView::mouseLeftReleased,this,&Cutting::leftReleasePoint);
        connect(cloud_view,&CloudView::mouseRightReleased,this,&Cutting::rightReleasePoint);
        connect(cloud_view,&CloudView::mouseMoved,this,&Cutting::movePoint);
        if(pick_type==0) {
            cloud_view->showInfo("Cuttingation [ON] (rectangular selection)",30,"info");
            cloud_view->showInfo("Left/Right click : set opposite corners",50,"info1");
        } else {
            cloud_view->showInfo("Cuttingation [ON] (polygonal selection)",30,"info");
            cloud_view->showInfo("Left click : add contour points / Right click : close",50,"info1");
        }
    } else {
        disconnect(cloud_view,&CloudView::mouseLeftPressed,this,&Cutting::leftPressPoint);
        disconnect(cloud_view,&CloudView::mouseLeftReleased,this,&Cutting::leftReleasePoint);
        disconnect(cloud_view,&CloudView::mouseRightReleased,this,&Cutting::rightReleasePoint);
        disconnect(cloud_view,&CloudView::mouseMoved,this,&Cutting::movePoint);
        cloud_tree->setEnabled(true);
        is_picking=false;
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        if(pick_type==0) {
            cloud_view->showInfo("Cuttingation [OFF] (rectangular selection)",30,"info");
            cloud_view->showInfo("Left/Right click : set opposite corners",50,"info1");
        } else {
            cloud_view->showInfo("Cuttingation [OFF] (polygonal selection)",30,"info");
            cloud_view->showInfo("Left click : add contour points / Right click : close",50,"info1");
        }
    }
}

void Cutting::reset()
{
    if(is_picking)
        this->start();
    Index index=cloud_tree->getSelectedIndex();
    cloud_tree->setCloudChecked(index,true);
    cloud_view->setInteractorEnable(true);
    cloud_view->removeShape("poly");
    cloud_view->removeShape("cut-box");
    cloud_view->removeCloud("cut");
    cloud_view->removeCloud("pre");
    points.clear();
}

void Cutting::leftPressPoint(const Point2D &pt)
{
    if(!pick_start) {
        points.clear();
        points.push_back(pt);
        pick_start=true;
    }
}

void Cutting::leftReleasePoint(const Point2D &pt)
{
    if(pick_start) {
        if((points.front().x==pt.x)&&(points.front().y==pt.y))
            return;
        else {
            if(pick_type==0) {
                //rectangle
                Point2D p1(points.front().x,pt.y);
                Point2D p2(pt.x,points.front().y);
                points.push_back(p1);
                points.push_back(pt);
                points.push_back(p2);
                cloud_view->addPolyLine(points,0,255,0,"poly");
                pick_start=false;
            } else {
                //polygon
                points.push_back(pt);
            }
        }
    }
}

void Cutting::rightReleasePoint(const Point2D &pt)
{
    if(pick_start) {
        if(pick_type==0) {
            //rectangle
            Point2D p1(points.front().x,pt.y);
            Point2D p2(pt.x,points.front().y);
            points.push_back(p1);
            points.push_back(pt);
            points.push_back(p2);
            pick_start=false;
        } else {
            //polygon
            if(points.size()==2){
                points.push_back(pt);
                cloud_view->addPolyLine(points,0,255,0,"poly");
                pick_start=false;
            } else if(points.size()>2){
                cloud_view->addPolyLine(points,0,255,0,"poly");
                pick_start=false;
            }
        }
    }

}

void Cutting::movePoint(const Point2D &pt)
{
    if(pick_start) {
        if(pick_type==0) {
            //rectangle
            std::vector<Point2D> pre_points=points;
            Point2D p1(pre_points.front().x,pt.y);
            Point2D p2(pt.x,pre_points.front().y);
            pre_points.push_back(p1);
            pre_points.push_back(pt);
            pre_points.push_back(p2);
            cloud_view->addPolyLine(pre_points,0,255,0,"poly");
        } else {
            //polygon
            std::vector<Point2D> pre_points=points;
            pre_points.push_back(pt);
            cloud_view->addPolyLine(pre_points,0,255,0,"poly");
        }
    }
}

void Cutting::cuttingCloud(bool selectIn)
{

    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(points.size()<3) {
        console->warning(tr("Please select an area!"));
        return;
    }
    if(is_picking)
        this->start();
    Index index=cloud_tree->getSelectedIndex();
    cloud_tree->setCloudChecked(index,false);
    cloud_view->setInteractorEnable(true);
    cloud_view->removeShape("poly");
    std::vector<int> indices=cloud_view->areaPick(points,selectedCloud,selectIn);
    cutted_cloud->clear();
    pcl::copyPointCloud(*selectedCloud,indices,*cutted_cloud);
    cutted_cloud->copyInfo(selectedCloud);
    cutted_cloud->computerBox();
    cloud_view->addCloud(cutted_cloud,"cut");
    cloud_view->addCube(cutted_cloud->box,"cut-box");
}

void Cutting::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloud_view->removeShape("info");
    cloud_view->removeShape("info1");
    cloud_tree->setExtendedSelection(true);
    return QDialog::closeEvent(event);
}

