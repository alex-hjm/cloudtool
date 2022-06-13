#include "color.h"
#include "ui_color.h"

Color::Color(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Color),
    fieldName(""),red(-1),green(-1),blue(-1)
{
    ui->setupUi(this);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Color::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Color::reset);
    ui->gridLayout->setSpacing(0);
    for (int row=0;row<5;row++){
        for(int column=0;column<10;column++){
            QPushButton *btnButton = new QPushButton();
            btnButton->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            btnButton->setFixedHeight(20);
            if(row==4 && column==9 ){
                btnButton->setText("+");
                btnButton->setStyleSheet(tr("QPushButton{border:none;background-color: rgb(239, 239, 239);}"
                                            "QPushButton:pressed{background-color: rgb(68, 190, 247);}"));
                connect(btnButton,&QPushButton::clicked,[=]{
                    QColor color=QColorDialog::getColor(Qt::white,this, tr("select color"));
                    red=color.red();green=color.green();blue=color.blue();
                    if(ui->rbtn_cloud->isChecked()){
                        emit cloudrgb(red,green,blue);
                    }
                    else if(ui->rbtn_back->isChecked()){
                        emit backgroundrgb(red,green,blue);
                    }
                    else if(ui->rbtn_model->isChecked()){
                        emit modelrgb(red,green,blue);
                    }
                });
            }
            else {
                int r=(Colors[row][column]).red();
                int g=(Colors[row][column]).green();
                int b=(Colors[row][column]).blue();
                btnButton->setStyleSheet(tr("QPushButton{border:none;background-color: rgb(%1, %2, %3);}"
                                            "QPushButton:pressed{background-color: rgb(68, 190, 247);}").arg(r).arg(g).arg(b));
                connect(btnButton,&QPushButton::clicked,[r,g,b,this]()
                {
                    if(ui->rbtn_cloud->isChecked()){
                        red=r;green=g;blue=b;
                        emit cloudrgb(r,g,b);
                    }
                    else if(ui->rbtn_back->isChecked()){
                        red=r;green=g;blue=b;
                        emit backgroundrgb(r,g,b);
                    }
                    else if(ui->rbtn_model->isChecked()){
                        red=r;green=g;blue=b;
                        emit modelrgb(r,g,b);
                    }
                });
            }
            ui->gridLayout->addWidget(btnButton,row,column);
        }
    }

    connect(ui->btn_x,&QPushButton::clicked,[=]{emit fieldname("x");});
    connect(ui->btn_y,&QPushButton::clicked,[=]{emit fieldname("y");});
    connect(ui->btn_z,&QPushButton::clicked,[=]{emit fieldname("z");});
    connect(ui->btn_r,&QPushButton::clicked,[=]{emit fieldname("random");});
    connect(ui->btn_c,&QPushButton::clicked,[=]{emit fieldname("curature");});
    connect(ui->btn_i,&QPushButton::clicked,[=]{emit fieldname("indensity");});
}

Color::~Color()
{
    delete ui;
}

void Color::init()
{
    connect(this,&Color::cloudrgb,[=](int r,int g,int b){
        std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
        if(selectedClouds.size()<=0){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        for (int i=0;i<selectedClouds.size();i++){
            cloudView->setCloudColor(selectedClouds[i].cloud,selectedClouds[i].id,r,g,b);
        }
    });

    connect(this,&Color::backgroundrgb,[=](int r,int g,int b){
        cloudView->setBackgroundColor(r,g,b);
    });

    connect(this,&Color::modelrgb,[=](int r,int g,int b){
        std::vector<Model> selectedModels=modelTree->getSelectedModels();
        if(selectedModels.size()<=0){
            console->warning(tr("Please select a model!"));
            return;
        }
        for (int i=0;i<selectedModels.size();i++){
            cloudView->setShapeColor(selectedModels[i].id,r,g,b);
        }
    });

    connect(this,&Color::fieldname,[=](std::string fieldname){
        fieldName=fieldname;
        std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
        if(selectedClouds.size()<=0){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        for (int i=0;i<selectedClouds.size();i++) {
            if(fieldname!="random")
                cloudView->setCloudColor(selectedClouds[i].cloud,selectedClouds[i].id,fieldname);
            else
                cloudView->setCloudColor(selectedClouds[i].cloud,selectedClouds[i].id,rand()%256,rand()%256,rand()%256);
        }
    });
}

void Color::apply()
{
    if((red!=-1)|fieldName!=""){
        if(ui->rbtn_cloud->isChecked()){
            std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
            std::vector<Index> indexs=cloudTree->getSelectedIndexs();
            if(selectedClouds.size()<=0){
                console->warning(tr("Please select a pointcloud!"));
                return;
            }
            for (int i=0;i<selectedClouds.size();i++){
                if(red!=-1 && fieldName=="") {
                    selectedClouds[i].cloud=com.setColor(selectedClouds[i].cloud,red,green,blue);
                    console->info(tr("The pointcloud ")+ selectedClouds[i].id.c_str()+tr(" color changed to %1 %2 %3").arg(red).arg(green).arg(blue));
                    cloudTree->updateCloud(indexs[i],selectedClouds[i]);
                }
                else if(fieldName!="") {
                    if(fieldName=="curature"|fieldName=="curature"|fieldName=="random")return;
                    selectedClouds[i].cloud=com.setColor(selectedClouds[i].cloud,fieldName);
                    console->info(tr("The pointcloud ")+selectedClouds[i].id.c_str()+tr(" color changed according to :")+fieldName.c_str());
                    cloudView->updateCloud(selectedClouds[i].cloud,selectedClouds[i].id);
                    cloudTree->updateCloud(indexs[i],selectedClouds[i]);
                }
            }
        }
        else if (ui->rbtn_model->isChecked()){
            //TODO:
            std::vector<Model> selectedModels=modelTree->getSelectedModels();
            std::vector<Index> indexs=modelTree->getSelectedIndexs();
            if(selectedModels.size()<=0){
                console->warning(tr("Please select a model!"));
                return;
            }
            for (int i=0;i<selectedModels.size();i++){
                selectedModels[i].r=red;
                selectedModels[i].g=green;
                selectedModels[i].b=blue;
                modelTree->updateModel(indexs[i],selectedModels[i]);
                console->info(tr("The model ")+selectedModels[i].id.c_str()+tr(" color changed to %1 %2 %3").arg(red).arg(green).arg(blue));
            }
        }
    }
    else{
        console->warning(tr("Please select a color!"));
    }
}

void Color::reset()
{
    if(ui->rbtn_back->isChecked())
        cloudView->resetBackgroundColor();
    if((red!=-1)|fieldName!=""){
        if(ui->rbtn_cloud->isChecked()) {
            std::vector<Cloud>  selectedClouds=cloudTree->getSelectedClouds();
            for (int i=0;i<selectedClouds.size();i++)
                cloudView->resetCloudColor(selectedClouds[i].cloud,selectedClouds[i].id);
        }
        if(ui->rbtn_model->isChecked()) {
            //TODO:
            std::vector<Model> selectedModels=modelTree->getSelectedModels();
            for (int i=0;i<selectedModels.size();i++){
               cloudView->setShapeColor(selectedModels[i].id,selectedModels[i].r,selectedModels[i].g,selectedModels[i].b);
            }
        }
    }
    red=green=blue=-1;fieldName="";
}

void Color::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}

