#include "selection.h"
#include "ui_selection.h"

Selection::Selection(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Selection)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Selection::preview);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Selection::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Selection::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Selection::close);

}

Selection::~Selection()
{
    delete ui;
}

void Selection::init()
{
    ui->dspin_max->setRange(-std::numeric_limits<double>::max (),std::numeric_limits<double>::max ());
    ui->dspin_min->setRange(-std::numeric_limits<double>::max (),std::numeric_limits<double>::max ());
}

void Selection::preview()
{
    selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    for(size_t i=0;i<finalClouds.size();i++) {
        cloudView->removeCloud("selected"+std::to_string(i));
    }
    finalClouds.clear();
    finalClouds=com.SelectCloud(selectedClouds,ui->cbox_feature->currentIndex(),ui->cbox_operation->currentIndex(),
                                ui->dspin_min->value(),ui->dspin_max->value());
    console->info(tr("%1 clouds has been selected").arg(finalClouds.size()));
    if(finalClouds.size()<=0)return;
    for(size_t i=0;i<finalClouds.size();i++) {
        if(finalClouds[i]->points.size()<=0)continue;
        cloudView->updateCloud(finalClouds[i],"selected"+std::to_string(i));
        cloudView->setCloudSize("selected"+std::to_string(i),5);
        cloudView->setCloudColor(finalClouds[i],"selected"+std::to_string(i),255,0,0);
    }
}

void Selection::apply()
{
    if(processTree->processEnable){




    }
}

void Selection::reset()
{
    for(size_t i=0;i<finalClouds.size();i++) {
        cloudView->removeCloud("selected"+std::to_string(i));
    }
    finalClouds.clear();
}

void Selection::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDialog::closeEvent(event);
}
