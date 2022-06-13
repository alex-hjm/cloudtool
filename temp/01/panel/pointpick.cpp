#include "pointpick.h"
#include "ui_pointpick.h"

PointPick::PointPick(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PointPick),is_picking(false),pick_cloud(new Cloud)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&PointPick::add);
    connect(ui->btn_start,&QPushButton::clicked,this,&PointPick::start);
    connect(ui->btn_reset,&QPushButton::clicked,this,&PointPick::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&PointPick::close);
}

PointPick::~PointPick()
{
    delete ui;
}

void PointPick::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    cloud_tree->setExtendedSelection(false);
    cloud_view->showInfo("PointPick [OFF]  (Left click : pick point)",30,"info");
}


void PointPick::start()
{
    if(!is_picking) {
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
        cloud_tree->setEnabled(false);
        is_picking=true;
        cloud_view->removeCloud("pick_cloud");
        cloud_view->removeShape("info1");
        cloud_view->removeShape("info2");
        cloud_view->removeShape("info3");
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));

        connect(cloud_view,&CloudView::mouseLeftReleased,this,&PointPick::leftReleasePoint);
        cloud_view->showInfo("PointPick [ON]  (Left click : pick point)",30,"info");
    } else {
        disconnect(cloud_view,&CloudView::mouseLeftReleased,this,&PointPick::leftReleasePoint);
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        cloud_tree->setEnabled(true);
        is_picking=false;
        cloud_view->removeCloud("pick_cloud");
        cloud_view->removeShape("info1");
        cloud_view->removeShape("info2");
        cloud_view->removeShape("info3");
        cloud_view->showInfo("PointPick [OFF]  (Left click : pick point)",30,"info");
    }
}

void PointPick::add()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(pick_cloud->empty()) {
        console->warning(tr("Please pick point first!"));
        return;
    }
    Index indexs=cloud_tree->getSelectedIndex();
    cloud_view->removeCloud("pick_cloud");
    cloud_view->removeShape("info1");
    cloud_view->removeShape("info2");
    cloud_view->removeShape("info3");
    pick_cloud->copyInfo(selectedCloud);
    pick_cloud->prefix("pick-");
    pick_cloud->update();
    cloud_tree->insertCloud(indexs.row,pick_cloud,true);
    cloud_view->updateCube(pick_cloud->box,pick_cloud->box_id);
    console->info(tr("Added successfully!"));
    pick_cloud->clear();
}

void PointPick::reset()
{
    if(is_picking)
        this->start();
    Index index=cloud_tree->getSelectedIndex();
    cloud_tree->setCloudChecked(index,true);
    ui->txt_output->clear();
    pick_cloud->clear();
    cloud_view->removeCloud("pick_cloud");
    cloud_view->removeShape("info1");
    cloud_view->removeShape("info2");
    cloud_view->removeShape("info3");
}

void PointPick::leftReleasePoint(const Point2D& pt)
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    int index=cloud_view->singlePick(pt);
    if(index==-1)return;
    PointXYZRGBN point=selectedCloud->points[index];
    QString xyz=tr("XYZ: %1 , %2 , %3 ").arg(point.x).arg(point.y).arg(point.z);
    QString rgb=tr("RGB: %1 , %2 , %3 ").arg(point.r).arg(point.g).arg(point.b);
    QString normal=tr("Normal: %1 , %2 , %3 ").arg(point.normal_x).arg(point.normal_y).arg(point.normal_z);
    ui->txt_output->clear();
    ui->txt_output->append(xyz);
    ui->txt_output->append(rgb);
    ui->txt_output->append(normal);
    cloud_view->showInfo(xyz.toStdString(),50,"info1");
    cloud_view->showInfo(rgb.toStdString(),70,"info2");
    cloud_view->showInfo(normal.toStdString(),90,"info3");
    pick_cloud->push_back(point);
    cloud_view->addCloud(pick_cloud,"pick_cloud");
    cloud_view->setCloudColor(pick_cloud,"pick_cloud",255,0,0);
    cloud_view->setCloudSize("pick_cloud",5);
}

void PointPick::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloud_view->removeShape("info");
    cloud_tree->setExtendedSelection(true);
    return QDialog::closeEvent(event);
}

