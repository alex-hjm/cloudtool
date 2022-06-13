#include "measure.h"
#include "ui_measure.h"

Measure::Measure(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Measure),start_pt(new Cloud),end_pt(new Cloud),
    is_measuring(false),pick_start(false)
{
    ui->setupUi(this);
    connect(ui->btn_start,&QPushButton::clicked,this,&Measure::start);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Measure::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Measure::close);
}

Measure::~Measure()
{
    delete ui;
}

void Measure::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    cloud_tree->setExtendedSelection(false);
    cloud_view->showInfo("Measure [OFF]  Distance:",30,"info");
    cloud_view->showInfo("Left click : pick  start/end point",50,"info1");
}


void Measure::start()
{
    if(!is_measuring) {
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
        is_measuring=true;
        cloud_view->removeShape("dis-arrow");
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));

        connect(cloud_view,&CloudView::mouseLeftPressed,this,&Measure::leftPressPoint);
        connect(cloud_view,&CloudView::mouseLeftReleased,this,&Measure::leftReleasePoint);
        connect(cloud_view,&CloudView::mouseRightReleased,this,&Measure::rightReleasePoint);
        cloud_view->showInfo("Measure [ON]  Distance:",30,"info");
        cloud_view->showInfo("Left click : pick  start/end point",50,"info1");
    } else {
        disconnect(cloud_view,&CloudView::mouseLeftPressed,this,&Measure::leftPressPoint);
        disconnect(cloud_view,&CloudView::mouseLeftReleased,this,&Measure::leftReleasePoint);
        disconnect(cloud_view,&CloudView::mouseRightReleased,this,&Measure::rightReleasePoint);
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        cloud_tree->setEnabled(true);
        is_measuring=false;
        cloud_view->removeShape("dis-arrow");
        cloud_view->removeCloud("startpoint");
        cloud_view->removeCloud("endpoint");
        cloud_view->showInfo("Measure [OFF]  Distance:",30,"info");
        cloud_view->showInfo("Left click : pick  start/end point",50,"info1");
    }
}


void Measure::reset()
{
    if(is_measuring)
        this->start();
    Index index=cloud_tree->getSelectedIndex();
    cloud_tree->setCloudChecked(index,true);
    cloud_view->removeShape("dis-arrow");
    cloud_view->removeCloud("startpoint");
    cloud_view->removeCloud("endpoint");
    ui->txt_distance->clear();
}

void Measure::leftPressPoint(const Point2D& pt)
{
    if(!pick_start) {
        current_pt=pt;
        start_pt->clear();
        Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
        int index=cloud_view->singlePick(pt);
        if(index==-1)return;
        start_pt->push_back(selectedCloud->points[index]);
        cloud_view->addCloud(start_pt,"startpoint");
        cloud_view->setCloudColor(start_pt,"startpoint",255,0,0);
        cloud_view->setCloudSize("startpoint",5);
        pick_start=true;
    }
}

void Measure::leftReleasePoint(const Point2D& pt)
{
    if(pick_start) {
        if((current_pt.x==pt.x)&&(current_pt.y==pt.y))
            return;
        else {
            end_pt->clear();
            Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
            int index=cloud_view->singlePick(pt);
            if(index==-1)return;
            end_pt->push_back(selectedCloud->points[index]);
            cloud_view->addCloud(end_pt,"endpoint");
            cloud_view->setCloudColor(end_pt,"endpoint",255,0,0);
            cloud_view->setCloudSize("endpoint",5);
            cloud_view->addArrow(end_pt->front(),start_pt->front(),0,255,0,true,"dis-arrow");
            float x=start_pt->front().x-end_pt->front().x;
            float y=start_pt->front().y-end_pt->front().y;
            float z=start_pt->front().z-end_pt->front().z;
            float distance=sqrt(x*x+y*y+z*z);
            ui->txt_distance->setText(QString::number(distance));
            std::string dis="Measure [ON]  Distance: "+std::to_string(distance)+" mm";
            cloud_view->showInfo(dis,30,"info");
            pick_start=false;
        }
    }
}

void Measure::rightReleasePoint(const Point2D& pt)
{
    if(pick_start) {
        if((current_pt.x==pt.x)&&(current_pt.y==pt.y))
            return;
        else {
            end_pt->clear();
            Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
            int index=cloud_view->singlePick(pt);
            if(index==-1)return;
            end_pt->push_back(selectedCloud->points[index]);
            cloud_view->addCloud(end_pt,"endpoint");
            cloud_view->setCloudColor(end_pt,"endpoint",255,0,0);
            cloud_view->setCloudSize("endpoint",5);
            cloud_view->addArrow(end_pt->front(),start_pt->front(),0,255,0,true,"dis-arrow");
            float x=start_pt->front().x-end_pt->front().x;
            float y=start_pt->front().y-end_pt->front().y;
            float z=start_pt->front().z-end_pt->front().z;
            float distance=sqrt(x*x+y*y+z*z);
            ui->txt_distance->setText(QString::number(distance));
            std::string dis="Measure [ON]  Distance: "+std::to_string(distance)+" mm";
            cloud_view->showInfo(dis,30,"info");
            pick_start=false;
        }

    }
}

void Measure::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloud_view->removeShape("info");
    cloud_view->removeShape("info1");
    cloud_tree->setExtendedSelection(true);
    return QDialog::closeEvent(event);
}

