#include "setpath.h"
#include "ui_setpath.h"

SetPath::SetPath(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::SetPath),affine(Eigen::Affine3f::Identity()),is_picking(false)
{
    ui->setupUi(this);
    connect(ui->btn_use_center,&QPushButton::clicked,this,&SetPath::useCenter);
    connect(ui->btn_pickpoint,&QPushButton::clicked,this,&SetPath::pickPoint);
    connect(ui->btn_preview,&QPushButton::clicked,this,&SetPath::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&SetPath::add);
    connect(ui->btn_clear,&QPushButton::clicked,this,&SetPath::clear);
    connect(ui->btn_reset,&QPushButton::clicked,this,&SetPath::reset);
    connect(ui->dspin_rx,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=Tool::AffineFromXYZEuler(ui->dspin_x->value(),ui->dspin_y->value(),ui->dspin_z->value(),
                                        value,ui->dspin_ry->value(),ui->dspin_rz->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_ry,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=Tool::AffineFromXYZEuler(ui->dspin_x->value(),ui->dspin_y->value(),ui->dspin_z->value(),
                                        ui->dspin_rx->value(),value,ui->dspin_rz->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_rz,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=Tool::AffineFromXYZEuler(ui->dspin_x->value(),ui->dspin_y->value(),ui->dspin_z->value(),
                                        ui->dspin_rx->value(),ui->dspin_ry->value(),value);
        emit affine3f(affine);
    });
    connect(ui->dspin_x,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=Tool::AffineFromXYZEuler(value,ui->dspin_y->value(),ui->dspin_z->value(),
                                        ui->dspin_rx->value(),ui->dspin_ry->value(),ui->dspin_rz->value());
        emit affine3f(affine);

    });
    connect(ui->dspin_y,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=Tool::AffineFromXYZEuler(ui->dspin_x->value(),value,ui->dspin_z->value(),
                                        ui->dspin_rx->value(),ui->dspin_ry->value(),ui->dspin_rz->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_z,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        affine=Tool::AffineFromXYZEuler(ui->dspin_x->value(),ui->dspin_y->value(),value,
                                        ui->dspin_rx->value(),ui->dspin_ry->value(),ui->dspin_rz->value());
        emit affine3f(affine);
    });
    connect(ui->check_settool,&QCheckBox::clicked,[=](bool checked)
    {
        if(checked)
        {
            ui->lineEdit_tool->setEnabled(true);
            ui->btn_use_center->setEnabled(false);
            ui->btn_pickpoint->setEnabled(false);
            this->adjustEnable(false);
        } else {
            ui->lineEdit_tool->setEnabled(false);
            ui->btn_use_center->setEnabled(true);
            ui->btn_pickpoint->setEnabled(true);
        }
    });
}

SetPath::~SetPath()
{
    delete ui;
}

void SetPath::init(Console *&co, CloudView *&cv, CloudTree *&ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    cloud_tree->setExtendedSelection(false);
    this->adjustEnable(false);
}

void SetPath::useCenter()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    this->adjustEnable(false);
    cloud_view->removeShape(selectedCloud->box_id);
    cloud_view->addCoord(selectedCloud->resolution*40,selectedCloud->box.affine,"pick_pos");
    float x,y,z,rx,ry,rz;
    pcl::getTranslationAndEulerAngles(selectedCloud->box.affine,x,y,z,rx,ry,rz);
    ui->dspin_x->setValue(x);
    ui->dspin_y->setValue(y);
    ui->dspin_z->setValue(z);
    ui->dspin_rx->setValue(rx/M_PI*180);
    ui->dspin_ry->setValue(ry/M_PI*180);
    ui->dspin_rz->setValue(rz/M_PI*180);
    this->adjustEnable(true);
}

void SetPath::pickPoint()
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
        cloud_view->removeCoord("pick_pos");
        cloud_tree->setEnabled(false);
        is_picking=true;
        this->adjustEnable(false);
        cloud_view->showInfo("Point Picking [ON] : Left click",50,"info1");
        ui->btn_pickpoint->setText("stop");
        ui->btn_pickpoint->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        connect(cloud_view,&CloudView::mouseLeftPressed,this,&SetPath::mouseLeftPressed);
    } else {
        disconnect(cloud_view,&CloudView::mouseLeftPressed,this,&SetPath::mouseLeftPressed);
        cloud_view->removeShape("info1");
        cloud_tree->setEnabled(true);
        is_picking=false;
        this->adjustEnable(true);
        ui->btn_pickpoint->setText("PickPoint");
        ui->btn_pickpoint->setIcon(QIcon(":/icon/resource/icon/start.svg"));
    }
}

void SetPath::preview()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(ui->check_settool->isChecked()) {
        cloud_view->showInfo("Path Points Num : "+std::to_string(selectedCloud->path_points.size()),30,"info");
        for(size_t i=0;i<selectedCloud->path_points.size();i++) {
            Eigen::Affine3f toolpath(selectedCloud->path_points[i]);
            float x,y,z,rx,ry,rz;
            pcl::getTranslationAndEulerAngles(toolpath,x,y,z,rx,ry,rz);
            cloud_view->showInfo("Tool Path : "+std::to_string(x)+" "+std::to_string(y)
                    +" "+std::to_string(z)+" "+std::to_string(rx/ M_PI * 180)
                    +" "+std::to_string(ry/ M_PI * 180)+" "+std::to_string(rz/ M_PI * 180),50,"info1");
        }
    } else {
        cloud_view->removeCoord("pick_pos");
        cloud_view->showInfo("Path Points Num : "+std::to_string(selectedCloud->path_points.size()),30,"info");
        for(size_t i=0;i<selectedCloud->path_points.size();i++) {
            Eigen::Matrix4f path_point=selectedCloud->path_points[i];
            Eigen::Affine3f path_point_aff(path_point);
            Eigen::Vector3f xyz=path_point.topRightCorner(3, 1);
            cloud_view->addCoord(60,path_point_aff,"path_"+std::to_string(i));
            cloud_view->addText3D("path_"+std::to_string(i),PointXYZRGBN(xyz[0],xyz[1],xyz[2],0,0,0),selectedCloud->resolution*10,0,255,255,"path_txt"+std::to_string(i));
        }
    }
}

void SetPath::add()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(!ui->check_settool->isChecked()) {
        selectedCloud->addPathPoint(affine.matrix());
        selectedCloud->is_tool_path=false;
        this->preview();
        cloud_view->showInfo("Path Points Num : "+std::to_string(selectedCloud->path_points.size()),30,"info");
    }
    else {
        QStringList textlist = ui->lineEdit_tool->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
        if (textlist.size() != 6) {
            console->warning(tr("Please input the correct format!"));
            return;
        }
        affine=Tool::AffineFromXYZEuler(textlist[0].toFloat(),textlist[1].toFloat(),textlist[2].toFloat(),
                textlist[3].toFloat(),textlist[4].toFloat(),textlist[5].toFloat());
        selectedCloud->addPathPoint(affine.matrix());
        selectedCloud->is_tool_path=true;
        cloud_view->showInfo("Path Points Num : "+std::to_string(selectedCloud->path_points.size()),30,"info");
        cloud_view->showInfo("Tool Path : "+std::to_string(textlist[0].toFloat())+" "+std::to_string(textlist[1].toFloat())
                +" "+std::to_string(textlist[2].toFloat())+" "+std::to_string(textlist[3].toFloat())
                +" "+std::to_string(textlist[4].toFloat())+" "+std::to_string(textlist[5].toFloat()),50,"info1");
    }
    console->info(tr("Added path point successfully !"));
}

void SetPath::clear()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->info(tr("Please select a pointcloud!"));
        return;
    }
    for(size_t i=0;i<selectedCloud->path_points.size();i++) {
        cloud_view->removeCoord("path_"+std::to_string(i));
        cloud_view->removeShape("path_txt"+std::to_string(i));
    }
    selectedCloud->clearPathPoints();
    console->warning(tr("Clear path points successfully !"));
    cloud_view->showInfo("Path Points Num : "+std::to_string(selectedCloud->path_points.size()),30,"info");
}


void SetPath::reset()
{
    if(is_picking)
        this->pickPoint();
    Index index=cloud_tree->getSelectedIndex();
    cloud_tree->setCloudChecked(index,true);
    this->adjustEnable(false);
    cloud_view->removeShape("info");
    cloud_view->removeShape("info1");
    cloud_view->removeCoord("pick_pos");
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    for(size_t i=0;i<selectedCloud->path_points.size();i++) {
        cloud_view->removeCoord("path_"+std::to_string(i));
        cloud_view->removeShape("path_txt"+std::to_string(i));
    }
}

void SetPath::mouseLeftPressed(const Point2D& pt)
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    int index=cloud_view->singlePick(pt);
    if(index==-1)return;
    PointXYZRGBN point=selectedCloud->points[index];
    cloud_view->addCoord(60,point.x,point.y,point.z,"pick_pos");
    ui->dspin_x->setValue(point.x);
    ui->dspin_y->setValue(point.y);
    ui->dspin_z->setValue(point.z);
    ui->dspin_rx->setValue(0);
    ui->dspin_ry->setValue(0);
    ui->dspin_rz->setValue(0);
}

void SetPath::adjustEnable(bool state)
{
    if(state) {
        connect(this,&SetPath::affine3f,this,&SetPath::adjustPickPos);
        ui->dspin_x->setEnabled(true);
        ui->dspin_y->setEnabled(true);
        ui->dspin_z->setEnabled(true);
        ui->dspin_rx->setEnabled(true);
        ui->dspin_ry->setEnabled(true);
        ui->dspin_rz->setEnabled(true);
    }
    else{
        disconnect(this,&SetPath::affine3f,this,&SetPath::adjustPickPos);
        ui->dspin_x->setEnabled(false);
        ui->dspin_y->setEnabled(false);
        ui->dspin_z->setEnabled(false);
        ui->dspin_rx->setEnabled(false);
        ui->dspin_ry->setEnabled(false);
        ui->dspin_rz->setEnabled(false);
    }
}

void SetPath::adjustPickPos(const Eigen::Affine3f & affine)
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    cloud_view->addCoord(selectedCloud->resolution*40,affine,"pick_pos");
}

void SetPath::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloud_tree->setExtendedSelection(true);
    return QDockWidget::closeEvent(event);
}
