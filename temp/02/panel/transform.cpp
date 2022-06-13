#include "transform.h"
#include "ui_transform.h"

Transform::Transform(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Transform),
    affine(Eigen::Affine3f::Identity())
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Transform::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Transform::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Transform::reset);
    ui->tabWidget->setCurrentIndex(0);
    //matrix
    connect(ui->txt_matrix,&QTextEdit::textChanged,[=]{
        bool success=false;
        affine=tool.AffineFromQString(ui->txt_matrix->toPlainText(),success);
        if(!success) {
            console->warning(tr("The transformation matrix format is wrong"));
            return;
        }
        if(ui->tabWidget->currentIndex()==0)
            emit Affine3f(affine);
    });
    //eulerAngle
    connect(ui->dspin_rx1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                       value,ui->dspin_ry1->value(),ui->dspin_rz1->value());
        if(ui->tabWidget->currentIndex()==1)
            emit Affine3f(affine);
    });
    connect(ui->dspin_ry1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                       ui->dspin_rx1->value(),value,ui->dspin_rz1->value());
        if(ui->tabWidget->currentIndex()==1)
            emit Affine3f(affine);
    });
    connect(ui->dspin_rz1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                       ui->dspin_rx1->value(),ui->dspin_ry1->value(),value);
        if(ui->tabWidget->currentIndex()==1)
            emit Affine3f(affine);
    });
    connect(ui->dspin_tx1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromXYZEuler(value,ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                       ui->dspin_rx1->value(),ui->dspin_ry1->value(),ui->dspin_rz1->value());
        if(ui->tabWidget->currentIndex()==1)
            emit Affine3f(affine);

    });
    connect(ui->dspin_ty1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromXYZEuler(ui->dspin_tx1->value(),value,ui->dspin_tz1->value(),
                                       ui->dspin_rx1->value(),ui->dspin_ry1->value(),ui->dspin_rz1->value());
        if(ui->tabWidget->currentIndex()==1)
            emit Affine3f(affine);
    });
    connect(ui->dspin_tz1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        affine=tool.AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),value,
                                       ui->dspin_rx1->value(),ui->dspin_ry1->value(),ui->dspin_rz1->value());
        if(ui->tabWidget->currentIndex()==1)
            emit Affine3f(affine);
    });
    //axisAngle
    connect(ui->dspin_angle,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromAxisAngle(value,ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                        ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        if(ui->tabWidget->currentIndex()==2)
            emit Affine3f(affine);
    });
    connect(ui->dspin_ax,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromAxisAngle(ui->dspin_angle->value(),value,ui->dspin_ay->value(),ui->dspin_az->value(),
                                        ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        if(ui->tabWidget->currentIndex()==2)
            emit Affine3f(affine);
    });
    connect(ui->dspin_ay,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),value,ui->dspin_az->value(),
                                        ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        if(ui->tabWidget->currentIndex()==2)
            emit Affine3f(affine);
    });
    connect(ui->dspin_az,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        affine=tool.AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),value,
                                        ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        if(ui->tabWidget->currentIndex()==2)
            emit Affine3f(affine);
    });
    connect(ui->dspin_tx2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        affine=tool.AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                        value,ui->dspin_ty2->value(),ui->dspin_tz2->value());
        if(ui->tabWidget->currentIndex()==2)
            emit Affine3f(affine);
    });
    connect(ui->dspin_ty2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        affine=tool.AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                        ui->dspin_tx2->value(),value,ui->dspin_tz2->value());
        if(ui->tabWidget->currentIndex()==2)
            emit Affine3f(affine);
    });
    connect(ui->dspin_tz2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        affine=tool.AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                        ui->dspin_tx2->value(),ui->dspin_ty2->value(),value);
        if(ui->tabWidget->currentIndex()==2)
            emit Affine3f(affine);
    });
    connect(ui->spin_decimals,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value) {
        disconnect(this,&Transform::Affine3f,this,&Transform::transCloud);
        Eigen::Matrix4f Martrix=affine.matrix();
        ui->txt_matrix->setText(tool.QStringFromMatrix(Martrix,value));
        ui->dspin_rx1->setDecimals(value);
        ui->dspin_ry1->setDecimals(value);
        ui->dspin_rz1->setDecimals(value);
        ui->dspin_tx1->setDecimals(value);
        ui->dspin_ty1->setDecimals(value);
        ui->dspin_tz1->setDecimals(value);
        ui->dspin_angle->setDecimals(value);
        ui->dspin_ax->setDecimals(value);
        ui->dspin_ay->setDecimals(value);
        ui->dspin_az->setDecimals(value);
        ui->dspin_tx2->setDecimals(value);
        ui->dspin_ty2->setDecimals(value);
        ui->dspin_tz2->setDecimals(value);
        ui->dspin_step->setDecimals(value);
        connect(this,&Transform::Affine3f,this,&Transform::transCloud);
    });
    connect(ui->dspin_step,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        ui->dspin_rx1->setSingleStep(value);
        ui->dspin_ry1->setSingleStep(value);
        ui->dspin_rz1->setSingleStep(value);
        ui->dspin_tx1->setSingleStep(value);
        ui->dspin_ty1->setSingleStep(value);
        ui->dspin_tz1->setSingleStep(value);
        ui->dspin_angle->setSingleStep(value);
        ui->dspin_ax->setSingleStep(value);
        ui->dspin_ay->setSingleStep(value);
        ui->dspin_az->setSingleStep(value);
        ui->dspin_tx2->setSingleStep(value);
        ui->dspin_ty2->setSingleStep(value);
        ui->dspin_tz2->setSingleStep(value);
    });
    ui->dspin_step->setValue(1);
    //singal
    connect(this,&Transform::Affine3f,[=](Eigen::Affine3f affine){
        Eigen::Matrix4f Martrix=affine.matrix();
        Eigen::Matrix3f rotation_matrix=Martrix.topLeftCorner(3,3);
        Eigen::Vector3f translation_vector(Martrix.topRightCorner(3,1));
        Eigen::Vector3f eulerAngle=rotation_matrix.eulerAngles(0,1,2);//r,p,y
        Eigen::AngleAxisf angleAxis;
        angleAxis.fromRotationMatrix(rotation_matrix);
        Eigen::Vector3f axis(angleAxis.axis());
        int index=ui->tabWidget->currentIndex();
        //matrix
        if(index!=0)
            ui->txt_matrix->setText(tool.QStringFromMatrix(Martrix,ui->spin_decimals->value()));
        //eulerAngle
        if(index!=1) {
            ui->dspin_rx1->setValue(eulerAngle[0]/ M_PI * 180);
            ui->dspin_ry1->setValue(eulerAngle[1]/ M_PI * 180);
            ui->dspin_rz1->setValue(eulerAngle[2]/ M_PI * 180);
            ui->dspin_tx1->setValue(translation_vector[0]);
            ui->dspin_ty1->setValue(translation_vector[1]);
            ui->dspin_tz1->setValue(translation_vector[2]);
        }
        //axisAngle
        if(index!=2) {
            ui->dspin_angle->setValue(angleAxis.angle()/ M_PI * 180);
            ui->dspin_ax->setValue(axis[0]);
            ui->dspin_ay->setValue(axis[1]);
            ui->dspin_az->setValue(axis[2]);
            ui->dspin_tx2->setValue(translation_vector[0]);
            ui->dspin_ty2->setValue(translation_vector[1]);
            ui->dspin_tz2->setValue(translation_vector[2]);
        }
    });
}

Transform::~Transform()
{
    delete ui;
}

void Transform::init()
{
    connect(this,&Transform::Affine3f,this,&Transform::transCloud);
    connect(cloudTree,&CloudTree::removedId,this,&Transform::removeCloud);
}

void Transform::add()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(affine.matrix()==Eigen::Matrix4f::Identity()) {
        console->warning(tr("Please transform the pointcloud first!"));
        return;
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].id+"-affine");
        CloudXYZRGBN::Ptr cloud(new CloudXYZRGBN);
        if(!ui->cbox_inverse->checkState())
            pcl::transformPointCloud(*selectedClouds[i].cloud,*cloud,affine);
        else
            pcl::transformPointCloud(*selectedClouds[i].cloud,*cloud,affine.inverse());
        Cloud trans_cloud(cloud,selectedClouds[i].fileInfo);
        trans_cloud.prefix("trans-");
        cloudTree->insertCloud(indexs[i].row,trans_cloud,true);
        cloudView->updateBoundingBox(trans_cloud.box,trans_cloud.boxid);
    }
}

void Transform::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].id+"-affine");
        if(!ui->cbox_inverse->checkState())
            pcl::transformPointCloud(*selectedClouds[i].cloud,*selectedClouds[i].cloud,affine);
        else
            pcl::transformPointCloud(*selectedClouds[i].cloud,*selectedClouds[i].cloud,affine.inverse());
        cloudTree->updateCloud(indexs[i],selectedClouds[i]);
        cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
    }
}

void Transform::reset()
{
    int index=ui->tabWidget->currentIndex();
    switch (index){
    case 0:
        affine=Eigen::Affine3f::Identity();
        ui->txt_matrix->setText(tool.QStringFromMatrix(affine.matrix(),ui->spin_decimals->value()));
        break;
    case 1:
        ui->dspin_rx1->setValue(0);
        ui->dspin_ry1->setValue(0);
        ui->dspin_rz1->setValue(0);
        ui->dspin_tx1->setValue(0);
        ui->dspin_ty1->setValue(0);
        ui->dspin_tz1->setValue(0);
        break;
    case 2:
        ui->dspin_angle->setValue(0);
        ui->dspin_ax->setValue(1);
        ui->dspin_ay->setValue(0);
        ui->dspin_az->setValue(0);
        ui->dspin_tx2->setValue(0);
        ui->dspin_ty2->setValue(0);
        ui->dspin_tz2->setValue(0);
        break;
    }
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].id+"-affine");
    }
}

void Transform::transCloud(Eigen::Affine3f affinein)
{
    affine=affinein;
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->updateCloud(selectedClouds[i].cloud,selectedClouds[i].id+"-affine");
        if(!ui->cbox_inverse->checkState())
            cloudView->updateCloudPose(selectedClouds[i].id+"-affine",affinein);
        else
            cloudView->updateCloudPose(selectedClouds[i].id+"-affine",affinein.inverse());
        cloudView->setCloudSize(selectedClouds[i].id+"-affine",3);
    }
}

void Transform::removeCloud(const string &id)
{
    cloudView->removeCloud(id+"-affine");
}

void Transform::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}
