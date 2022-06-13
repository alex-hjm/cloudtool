#include "coords.h"
#include "ui_coords.h"

Coords::Coords(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Coords)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Coords::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Coords::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Coords::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Coords::close);
    connect(ui->btn_add_coord,&QPushButton::clicked,this,&Coords::addCoord);
    connect(ui->btn_close_coord,&QPushButton::clicked,this,&Coords::closeCoord);
    ui->txt_matrix->hide();
    ui->btn_add_coord->hide();
    ui->btn_close_coord->hide();
    ui->label->hide();
    ui->lineEdit_coord_id->hide();
    ui->check_inverse->hide();
    this->setFixedSize(202,32);
}

Coords::~Coords()
{
    delete ui;
}

void Coords::init(Console *& co, CloudView *&cv , CloudTree *&ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    connect(ui->dspin_scale,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
        if(selectedClouds.empty())
            cloud_view->updateCoord(value,"coords");
        else
            for(auto &i:selectedClouds)
                if(cloud_view->contains(i->id+"-coord"))
                    cloud_view->updateCoord(value,i->box.affine,i->id+"-coord");
                else
                    cloud_view->updateCoord(value,"coords");
    });
    cloud_view->addCoord(ui->dspin_scale->value(),"coords");
    connect(cloud_tree,&CloudTree::removedId,this,&Coords::removeCoords);

}

void Coords::addCoord()
{
    if(ui->lineEdit_coord_id->text().isEmpty()){
        console->warning(tr("Please input a id!"));
        return;
    }
    std::string id=ui->lineEdit_coord_id->text().toStdString();
    if(cloud_view->contains(id)){
        console->warning(tr("The id ")+id.c_str()+tr(" already exists! Please rename a different id and retry."));
        return;
    }
    bool success=false;
    Eigen::Affine3f affine=Tool::AffineFromQString(ui->txt_matrix->toPlainText(),success);
    if(!success) {
        console->warning(tr("The transformation matrix format is wrong"));
        return;
    }
    if(!ui->check_inverse->isChecked())
        cloud_view->addCoord(ui->dspin_scale->value(),affine,id);
    else
        cloud_view->addCoord(ui->dspin_scale->value(),affine.inverse(),id);
}

void Coords::closeCoord()
{
    ui->txt_matrix->hide();
    ui->btn_add_coord->hide();
    ui->btn_close_coord->hide();
    ui->label->hide();
    ui->lineEdit_coord_id->hide();
    ui->check_inverse->hide();
    this->setFixedSize(202,32);
    cloud_view->removeAllCoords();
    cloud_view->addCoord(ui->dspin_scale->value(),"coords");
}

void Coords::add()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        this->setFixedSize(202,144);
        ui->txt_matrix->show();
        ui->btn_add_coord->show();
        ui->btn_close_coord->show();
        ui->label->show();
        ui->lineEdit_coord_id->show();
        ui->check_inverse->show();
        return;
    }
    for(auto &i:selectedClouds)
        cloud_view->addCoord(ui->dspin_scale->value(),i->box.affine,i->id+"-coord");
    console->info(tr("Added successfully!"));
}

void Coords::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    for(auto &i:selectedClouds) {
        cloud_view->removeCoord(i->id+"-coord");
        Eigen::Affine3f affine=pcl::getTransformation(i->center()[0],i->center()[1],i->center()[2],0,0,0);
        pcl::transformPointCloud(*i,*i,affine.inverse());
        i->computerBox();
        cloud_view->updateCloud(i,i->id);
        cloud_view->updateCube(i->box,i->box_id);
    }
    console->info(tr("Applied successfully!"));
}

void Coords::reset()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    for(auto &i:selectedClouds)
        cloud_view->removeCoord(i->id+"-coord");
    cloud_view->addCoord(ui->dspin_scale->value(),"coords");
}

void Coords::removeCoords(const std::string &id)
{
    cloud_view->removeCoord(id+"-coord");
}

void Coords::closeEvent(QCloseEvent * event)
{
    this->reset();
    cloud_view->removeCoord("coords");
    return QDialog::closeEvent(event);
}
