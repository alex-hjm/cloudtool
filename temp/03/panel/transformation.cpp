#include "transformation.h"
#include "ui_transformation.h"

Transformation::Transformation(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Transformation),affine(Eigen::Affine3f::Identity())
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Transformation::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Transformation::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Transformation::reset);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Transformation::preview);
    ui->tabWidget->setCurrentIndex(0);
    //matrix
    connect(ui->txt_matrix,&QTextEdit::textChanged,[=](){
        if(!ui->tabWidget->currentIndex()==0)return;
        bool success=false;
        affine=Tool::AffineFromQString(ui->txt_matrix->toPlainText(),success);
        if(!success) {
            console->warning(tr("The transformation matrix format is wrong"));
            return;
        }
        emit affine3f(affine);
    });
    //eulerAngle
    connect(ui->dspin_rx1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->tabWidget->currentIndex()==1)return;
        affine=Tool::AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                        value,ui->dspin_ry1->value(),ui->dspin_rz1->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_ry1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->tabWidget->currentIndex()==1)return;
        affine=Tool::AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                        ui->dspin_rx1->value(),value,ui->dspin_rz1->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_rz1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->tabWidget->currentIndex()==1)return;
        affine=Tool::AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                        ui->dspin_rx1->value(),ui->dspin_ry1->value(),value);
        emit affine3f(affine);
    });
    connect(ui->dspin_tx1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->tabWidget->currentIndex()==1)return;
        affine=Tool::AffineFromXYZEuler(value,ui->dspin_ty1->value(),ui->dspin_tz1->value(),
                                        ui->dspin_rx1->value(),ui->dspin_ry1->value(),ui->dspin_rz1->value());
        emit affine3f(affine);

    });
    connect(ui->dspin_ty1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->tabWidget->currentIndex()==1)return;
        affine=Tool::AffineFromXYZEuler(ui->dspin_tx1->value(),value,ui->dspin_tz1->value(),
                                        ui->dspin_rx1->value(),ui->dspin_ry1->value(),ui->dspin_rz1->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_tz1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        if(!ui->tabWidget->currentIndex()==1)return;
        affine=Tool::AffineFromXYZEuler(ui->dspin_tx1->value(),ui->dspin_ty1->value(),value,
                                        ui->dspin_rx1->value(),ui->dspin_ry1->value(),ui->dspin_rz1->value());
        if(ui->tabWidget->currentIndex()==1)
            emit affine3f(affine);
    });
    connect(ui->txt_xyzeuler,&QLineEdit::textChanged,[=](const QString&text){
        if(!ui->tabWidget->currentIndex()==1)return;
        QStringList textlist = text.split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
        if (textlist.size() != 6) return;
        ui->dspin_tx1->setValue(textlist[0].toFloat());
        ui->dspin_ty1->setValue(textlist[1].toFloat());
        ui->dspin_tz1->setValue(textlist[2].toFloat());
        ui->dspin_rx1->setValue(textlist[3].toFloat());
        ui->dspin_ry1->setValue(textlist[4].toFloat());
        ui->dspin_rz1->setValue(textlist[5].toFloat());
    });
    //axisAngle
    connect(ui->dspin_angle,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(ui->tabWidget->currentIndex()!=2)return;
        affine=Tool::AffineFromAxisAngle(value,ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                         ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_ax,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(ui->tabWidget->currentIndex()!=2)return;
        affine=Tool::AffineFromAxisAngle(ui->dspin_angle->value(),value,ui->dspin_ay->value(),ui->dspin_az->value(),
                                         ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_ay,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(ui->tabWidget->currentIndex()!=2)return;
        affine=Tool::AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),value,ui->dspin_az->value(),
                                         ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_az,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        if(ui->tabWidget->currentIndex()!=2)return;
        affine=Tool::AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),value,
                                         ui->dspin_tx2->value(),ui->dspin_ty2->value(),ui->dspin_tz2->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_tx2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        if(ui->tabWidget->currentIndex()!=2)return;
        affine=Tool::AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                         value,ui->dspin_ty2->value(),ui->dspin_tz2->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_ty2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        if(ui->tabWidget->currentIndex()!=2)return;
        affine=Tool::AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                         ui->dspin_tx2->value(),value,ui->dspin_tz2->value());
        emit affine3f(affine);
    });
    connect(ui->dspin_tz2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(ui->tabWidget->currentIndex()!=2)return;
        affine=Tool::AffineFromAxisAngle(ui->dspin_angle->value(),ui->dspin_ax->value(),ui->dspin_ay->value(),ui->dspin_az->value(),
                                         ui->dspin_tx2->value(),ui->dspin_ty2->value(),value);
        emit affine3f(affine);
    });
    //singal
    connect(this,&Transformation::affine3f,[=](const Eigen::Affine3f &affine){
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
            ui->txt_matrix->setText(Tool::QStringFromMatrix(Martrix,3));
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

Transformation::~Transformation()
{
    delete ui;
}

void Transformation::init(Console* &co,CloudView* &cv,CloudTree* &ct,ProcessTree* pt)
{
    console=co;cloud_view=cv;cloud_tree=ct;process_tree=pt;
    connect(this,&Transformation::affine3f,[=]{
        if(!ui->check_refresh->isChecked())return;
        this->preview();
    });
    connect(cloud_tree,&CloudTree::removedId,this,&Transformation::removeCloud);
}

void Transformation::preview()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    for(auto &i:selectedClouds) {
        cloud_view->addCloud(i,i->id+"-affine");
        if(!ui->cbox_inverse->checkState())
            cloud_view->updateCloudPose(i->id+"-affine",affine);
        else
            cloud_view->updateCloudPose(i->id+"-affine",affine.inverse());
        cloud_view->setCloudSize(i->id+"-affine",3);
    }
}

void Transformation::add()
{
    if(process_tree->enable()){
        Process::Ptr transformation(new Process);
        transformation->id="Transformation";
        transformation->type=process_transformation;
        transformation->icon=QIcon(":/icon/resource/icon/transformation.svg");
        transformation->affine=affine;
        process_tree->insertProcess(transformation);
        return;
    }
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(affine.matrix()==Eigen::Matrix4f::Identity()) {
        console->warning(tr("Please transform the pointcloud first!"));
        return;
    }
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_view->removeCloud(selectedClouds[i]->id+"-affine");
        Cloud::Ptr trans_cloud(new Cloud);
        trans_cloud->copyInfo(selectedClouds[i]);
        if(!ui->cbox_inverse->checkState())
            pcl::transformPointCloud(*selectedClouds[i],*trans_cloud,affine);
        else
            pcl::transformPointCloud(*selectedClouds[i],*trans_cloud,affine.inverse());
        trans_cloud->prefix("trans-");
        trans_cloud->update();
        cloud_tree->insertCloud(indexs[i].row,trans_cloud,true);
        cloud_view->addCube(trans_cloud->box,trans_cloud->box_id);
    }
    console->info(tr("Added successfully!"));
    affine.setIdentity();
}

void Transformation::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    for(auto &i:selectedClouds) {
        cloud_view->removeCloud(i->id+"-affine");
        if(!ui->cbox_inverse->checkState())
            pcl::transformPointCloud(*i,*i,affine);
        else
            pcl::transformPointCloud(*i,*i,affine.inverse());
        i->update();
        cloud_view->updateCloud(i,i->id);
        cloud_view->updateCube(i->box,i->box_id);
    }
    console->info(tr("Applied successfully!"));
    affine.setIdentity();
}

void Transformation::reset()
{
    switch (ui->tabWidget->currentIndex()){
    case 0:
        affine=Eigen::Affine3f::Identity();
        ui->txt_matrix->setText(Tool::QStringFromMatrix(affine.matrix(),3));
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
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    for(auto &i:selectedClouds)
        cloud_view->removeCloud(i->id+"-affine");
}

void Transformation::removeCloud(const std::string &id)
{
    cloud_view->removeCloud(id+"-affine");
}

void Transformation::closeEvent(QCloseEvent *event)
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    for(auto &i:selectedClouds)
        cloud_view->removeCloud(i->id+"-affine");
    return QDockWidget::closeEvent(event);
}
