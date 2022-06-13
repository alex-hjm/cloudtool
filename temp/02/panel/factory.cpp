#include "factory.h"
#include "ui_factory.h"

Factory::Factory(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Factory),
    isAdjustable(true)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Factory::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Factory::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Factory::reset);

    ui->widget_models->setCurrentIndex(0);
}

Factory::~Factory()
{
    delete ui;
}

void Factory::init()
{
    modelTree->show();
    modelTree->setExtendedSelection(false);
    //plane
    connect(ui->dspin_plane_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_plane_2->value();
        coefs.values[2]=ui->dspin_plane_3->value();
        coefs.values[3]=ui->dspin_plane_4->value();
        coefs.values[4]=ui->dspin_plane_5->value();
        coefs.values[5]=ui->dspin_plane_6->value();
        coefs.values[6]=ui->dspin_plane_7->value();
        if(isAdjustable)
            emit planeCoefficients(coefs);
    });
    connect(ui->dspin_plane_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_plane_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_plane_3->value();
        coefs.values[3]=ui->dspin_plane_4->value();
        coefs.values[4]=ui->dspin_plane_5->value();
        coefs.values[5]=ui->dspin_plane_6->value();
        coefs.values[6]=ui->dspin_plane_7->value();
        if(isAdjustable)
            emit planeCoefficients(coefs);
    });
    connect(ui->dspin_plane_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_plane_1->value();
        coefs.values[1]=ui->dspin_plane_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_plane_4->value();
        coefs.values[4]=ui->dspin_plane_5->value();
        coefs.values[5]=ui->dspin_plane_6->value();
        coefs.values[6]=ui->dspin_plane_7->value();
        if(isAdjustable)
            emit planeCoefficients(coefs);
    });
    connect(ui->dspin_plane_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_plane_1->value();
        coefs.values[1]=ui->dspin_plane_2->value();
        coefs.values[2]=ui->dspin_plane_3->value();
        coefs.values[3]=value;
        coefs.values[4]=ui->dspin_plane_5->value();
        coefs.values[5]=ui->dspin_plane_6->value();
        coefs.values[6]=ui->dspin_plane_7->value();
        if(isAdjustable)
            emit planeCoefficients(coefs);
    });
    connect(ui->dspin_plane_5,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_plane_1->value();
        coefs.values[1]=ui->dspin_plane_2->value();
        coefs.values[2]=ui->dspin_plane_3->value();
        coefs.values[3]=ui->dspin_plane_4->value();
        coefs.values[4]=value;
        coefs.values[5]=ui->dspin_plane_6->value();
        coefs.values[6]=ui->dspin_plane_7->value();
        if(isAdjustable)
            emit planeCoefficients(coefs);
    });
    connect(ui->dspin_plane_6,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_plane_1->value();
        coefs.values[1]=ui->dspin_plane_2->value();
        coefs.values[2]=ui->dspin_plane_3->value();
        coefs.values[3]=ui->dspin_plane_4->value();
        coefs.values[4]=ui->dspin_plane_5->value();
        coefs.values[5]=value;
        coefs.values[6]=ui->dspin_plane_7->value();
        if(isAdjustable)
            emit planeCoefficients(coefs);
    });

    connect(ui->dspin_plane_7,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_plane_1->value();
        coefs.values[1]=ui->dspin_plane_2->value();
        coefs.values[2]=ui->dspin_plane_3->value();
        coefs.values[3]=ui->dspin_plane_4->value();
        coefs.values[4]=ui->dspin_plane_5->value();
        coefs.values[5]=ui->dspin_plane_6->value();
        coefs.values[6]=value;
        if(isAdjustable)
            emit planeCoefficients(coefs);
    });

    connect(this,&Factory::planeCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Plane)){
            model.type=Plane;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    //sphere
    connect(ui->dspin_sphere_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_sphere_2->value();
        coefs.values[2]=ui->dspin_sphere_3->value();
        coefs.values[3]=ui->dspin_sphere_4->value();
        if(isAdjustable)emit sphereCoefficients(coefs);
    });
    connect(ui->dspin_sphere_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=ui->dspin_sphere_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_sphere_3->value();
        coefs.values[3]=ui->dspin_sphere_4->value();
        if(isAdjustable)emit sphereCoefficients(coefs);
    });
    connect(ui->dspin_sphere_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=ui->dspin_sphere_1->value();
        coefs.values[1]=ui->dspin_sphere_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_sphere_4->value();
        if(isAdjustable)emit sphereCoefficients(coefs);
    });
    connect(ui->dspin_sphere_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=ui->dspin_sphere_1->value();
        coefs.values[1]=ui->dspin_sphere_2->value();
        coefs.values[2]=ui->dspin_sphere_3->value();
        coefs.values[3]=value;
        if(isAdjustable)emit sphereCoefficients(coefs);
    });

    connect(this,&Factory::sphereCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Sphere)){
            model.type=Sphere;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    //line
    connect(ui->dspin_line_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_line_2->value();
        coefs.values[2]=ui->dspin_line_3->value();
        coefs.values[3]=ui->dspin_line_4->value();
        coefs.values[4]=ui->dspin_line_5->value();
        coefs.values[5]=ui->dspin_line_6->value();
        if(isAdjustable)emit lineCoefficients(coefs);
    });
    connect(ui->dspin_line_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_line_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_line_3->value();
        coefs.values[3]=ui->dspin_line_4->value();
        coefs.values[4]=ui->dspin_line_5->value();
        coefs.values[5]=ui->dspin_line_6->value();
        if(isAdjustable)emit lineCoefficients(coefs);
    });
    connect(ui->dspin_line_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_line_1->value();
        coefs.values[1]=ui->dspin_line_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_line_4->value();
        coefs.values[4]=ui->dspin_line_5->value();
        coefs.values[5]=ui->dspin_line_6->value();
        if(isAdjustable)emit lineCoefficients(coefs);
    });
    connect(ui->dspin_line_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_line_1->value();
        coefs.values[1]=ui->dspin_line_2->value();
        coefs.values[2]=ui->dspin_line_3->value();
        coefs.values[3]=value;
        coefs.values[4]=ui->dspin_line_5->value();
        coefs.values[5]=ui->dspin_line_6->value();
        if(isAdjustable)emit lineCoefficients(coefs);
    });
    connect(ui->dspin_line_5,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_line_1->value();
        coefs.values[1]=ui->dspin_line_2->value();
        coefs.values[2]=ui->dspin_line_3->value();
        coefs.values[3]=ui->dspin_line_4->value();
        coefs.values[4]=value;
        coefs.values[5]=ui->dspin_line_6->value();
        if(isAdjustable)emit lineCoefficients(coefs);
    });
    connect(ui->dspin_line_6,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_line_1->value();
        coefs.values[1]=ui->dspin_line_2->value();
        coefs.values[2]=ui->dspin_line_3->value();
        coefs.values[3]=ui->dspin_line_4->value();
        coefs.values[4]=ui->dspin_line_5->value();
        coefs.values[5]=value;
        if(isAdjustable)emit lineCoefficients(coefs);
    });

    connect(this,&Factory::lineCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Line)){
            model.type=Line;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    //cyplinder
    connect(ui->dspin_cyplinder_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_cyplinder_2->value();
        coefs.values[2]=ui->dspin_cyplinder_3->value();
        coefs.values[3]=ui->dspin_cyplinder_4->value();
        coefs.values[4]=ui->dspin_cyplinder_5->value();
        coefs.values[5]=ui->dspin_cyplinder_6->value();
        coefs.values[6]=ui->dspin_cyplinder_7->value();
        if(isAdjustable)emit cyplinderCoefficients(coefs);
    });
    connect(ui->dspin_cyplinder_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cyplinder_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_cyplinder_3->value();
        coefs.values[3]=ui->dspin_cyplinder_4->value();
        coefs.values[4]=ui->dspin_cyplinder_5->value();
        coefs.values[5]=ui->dspin_cyplinder_6->value();
        coefs.values[6]=ui->dspin_cyplinder_7->value();
        if(isAdjustable)emit cyplinderCoefficients(coefs);
    });
    connect(ui->dspin_cyplinder_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cyplinder_1->value();
        coefs.values[1]=ui->dspin_cyplinder_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_cyplinder_4->value();
        coefs.values[4]=ui->dspin_cyplinder_5->value();
        coefs.values[5]=ui->dspin_cyplinder_6->value();
        coefs.values[6]=ui->dspin_cyplinder_7->value();
        if(isAdjustable)emit cyplinderCoefficients(coefs);
    });
    connect(ui->dspin_cyplinder_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cyplinder_1->value();
        coefs.values[1]=ui->dspin_cyplinder_2->value();
        coefs.values[2]=ui->dspin_cyplinder_3->value();
        coefs.values[3]=value;
        coefs.values[4]=ui->dspin_cyplinder_5->value();
        coefs.values[5]=ui->dspin_cyplinder_6->value();
        coefs.values[6]=ui->dspin_cyplinder_7->value();
        if(isAdjustable)emit cyplinderCoefficients(coefs);
    });
    connect(ui->dspin_cyplinder_5,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cyplinder_1->value();
        coefs.values[1]=ui->dspin_cyplinder_2->value();
        coefs.values[2]=ui->dspin_cyplinder_3->value();
        coefs.values[3]=ui->dspin_cyplinder_4->value();
        coefs.values[4]=value;
        coefs.values[5]=ui->dspin_cyplinder_6->value();
        coefs.values[6]=ui->dspin_cyplinder_7->value();
        if(isAdjustable)emit cyplinderCoefficients(coefs);
    });
    connect(ui->dspin_cyplinder_6,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cyplinder_1->value();
        coefs.values[1]=ui->dspin_cyplinder_2->value();
        coefs.values[2]=ui->dspin_cyplinder_3->value();
        coefs.values[3]=ui->dspin_cyplinder_4->value();
        coefs.values[4]=ui->dspin_cyplinder_5->value();
        coefs.values[5]=value;
        coefs.values[6]=ui->dspin_cyplinder_7->value();
        if(isAdjustable)emit cyplinderCoefficients(coefs);
    });
    connect(ui->dspin_cyplinder_7,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cyplinder_1->value();
        coefs.values[1]=ui->dspin_cyplinder_2->value();
        coefs.values[2]=ui->dspin_cyplinder_3->value();
        coefs.values[3]=ui->dspin_cyplinder_4->value();
        coefs.values[4]=ui->dspin_cyplinder_5->value();
        coefs.values[5]=ui->dspin_cyplinder_6->value();
        coefs.values[6]=value;
        if(isAdjustable)emit cyplinderCoefficients(coefs);
    });

    connect(this,&Factory::cyplinderCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Cylinder)){
            model.type=Cylinder;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    //circle
    connect(ui->dspin_circle_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(3);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_circle_2->value();
        coefs.values[2]=ui->dspin_circle_3->value();
        if(isAdjustable)emit circleCoefficients(coefs);
    });
    connect(ui->dspin_circle_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(3);
        coefs.values[0]=ui->dspin_circle_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_circle_3->value();
        if(isAdjustable)emit circleCoefficients(coefs);
    });
    connect(ui->dspin_circle_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(3);
        coefs.values[0]=ui->dspin_circle_1->value();
        coefs.values[1]=ui->dspin_circle_2->value();
        coefs.values[2]=value;
        if(isAdjustable)emit circleCoefficients(coefs);
    });

    connect(this,&Factory::circleCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Circle)){
            model.type=Circle;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    //cone
    connect(ui->dspin_cone_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_cone_2->value();
        coefs.values[2]=ui->dspin_cone_3->value();
        coefs.values[3]=ui->dspin_cone_4->value();
        coefs.values[4]=ui->dspin_cone_5->value();
        coefs.values[5]=ui->dspin_cone_6->value();
        coefs.values[6]=ui->dspin_cone_7->value();
        if(isAdjustable)emit coneCoefficients(coefs);
    });
    connect(ui->dspin_cone_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cone_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_cone_3->value();
        coefs.values[3]=ui->dspin_cone_4->value();
        coefs.values[4]=ui->dspin_cone_5->value();
        coefs.values[5]=ui->dspin_cone_6->value();
        coefs.values[6]=ui->dspin_cone_7->value();
        if(isAdjustable)emit coneCoefficients(coefs);
    });
    connect(ui->dspin_cone_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cone_1->value();
        coefs.values[1]=ui->dspin_cone_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_cone_4->value();
        coefs.values[4]=ui->dspin_cone_5->value();
        coefs.values[5]=ui->dspin_cone_6->value();
        coefs.values[6]=ui->dspin_cone_7->value();
        if(isAdjustable)emit coneCoefficients(coefs);
    });
    connect(ui->dspin_cone_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cone_1->value();
        coefs.values[1]=ui->dspin_cone_2->value();
        coefs.values[2]=ui->dspin_cone_3->value();
        coefs.values[3]=value;
        coefs.values[4]=ui->dspin_cone_5->value();
        coefs.values[5]=ui->dspin_cone_6->value();
        coefs.values[6]=ui->dspin_cone_7->value();
        if(isAdjustable)emit coneCoefficients(coefs);
    });
    connect(ui->dspin_cone_5,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cone_1->value();
        coefs.values[1]=ui->dspin_cone_2->value();
        coefs.values[2]=ui->dspin_cone_3->value();
        coefs.values[3]=ui->dspin_cone_4->value();
        coefs.values[4]=value;
        coefs.values[5]=ui->dspin_cone_6->value();
        coefs.values[6]=ui->dspin_cone_7->value();
        if(isAdjustable)emit coneCoefficients(coefs);
    });
    connect(ui->dspin_cone_6,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cone_1->value();
        coefs.values[1]=ui->dspin_cone_2->value();
        coefs.values[2]=ui->dspin_cone_3->value();
        coefs.values[3]=ui->dspin_cone_4->value();
        coefs.values[4]=ui->dspin_cone_5->value();
        coefs.values[5]=value;
        coefs.values[6]=ui->dspin_cone_7->value();
        if(isAdjustable)emit coneCoefficients(coefs);
    });
    connect(ui->dspin_cone_7,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(7);
        coefs.values[0]=ui->dspin_cone_1->value();
        coefs.values[1]=ui->dspin_cone_2->value();
        coefs.values[2]=ui->dspin_cone_3->value();
        coefs.values[3]=ui->dspin_cone_4->value();
        coefs.values[4]=ui->dspin_cone_5->value();
        coefs.values[5]=ui->dspin_cone_6->value();
        coefs.values[6]=value;
        if(isAdjustable)emit coneCoefficients(coefs);
    });

    connect(this,&Factory::coneCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Cone)){
            model.type=Cone;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    //cube
    connect(ui->dspin_cube_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_cube_2->value();
        coefs.values[2]=ui->dspin_cube_3->value();
        coefs.values[3]=ui->dspin_cube_4->value();
        coefs.values[4]=ui->dspin_cube_5->value();
        coefs.values[5]=ui->dspin_cube_6->value();
        if(isAdjustable)emit cubeCoefficients(coefs);
    });
    connect(ui->dspin_cube_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_cube_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_cube_3->value();
        coefs.values[3]=ui->dspin_cube_4->value();
        coefs.values[4]=ui->dspin_cube_5->value();
        coefs.values[5]=ui->dspin_cube_6->value();
        if(isAdjustable)emit cubeCoefficients(coefs);
    });
    connect(ui->dspin_cube_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_cube_1->value();
        coefs.values[1]=ui->dspin_cube_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_cube_4->value();
        coefs.values[4]=ui->dspin_cube_5->value();
        coefs.values[5]=ui->dspin_cube_6->value();
        if(isAdjustable)emit cubeCoefficients(coefs);
    });
    connect(ui->dspin_cube_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_cube_1->value();
        coefs.values[1]=ui->dspin_cube_2->value();
        coefs.values[2]=ui->dspin_cube_3->value();
        coefs.values[3]=value;
        coefs.values[4]=ui->dspin_cube_5->value();
        coefs.values[5]=ui->dspin_cube_6->value();
        if(isAdjustable)emit cubeCoefficients(coefs);
    });
    connect(ui->dspin_cube_5,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_cube_1->value();
        coefs.values[1]=ui->dspin_cube_2->value();
        coefs.values[2]=ui->dspin_cube_3->value();
        coefs.values[3]=ui->dspin_cube_4->value();
        coefs.values[4]=value;
        coefs.values[5]=ui->dspin_cube_6->value();
        if(isAdjustable)emit cubeCoefficients(coefs);
    });
    connect(ui->dspin_cube_6,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_cube_1->value();
        coefs.values[1]=ui->dspin_cube_2->value();
        coefs.values[2]=ui->dspin_cube_3->value();
        coefs.values[3]=ui->dspin_cube_4->value();
        coefs.values[4]=ui->dspin_cube_5->value();
        coefs.values[5]=value;
        if(isAdjustable)emit cubeCoefficients(coefs);
    });

    connect(this,&Factory::cubeCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Cube)){
            model.type=Cube;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    //arrow
    connect(ui->dspin_arrow_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_arrow_2->value();
        coefs.values[2]=ui->dspin_arrow_3->value();
        coefs.values[3]=ui->dspin_arrow_4->value();
        coefs.values[4]=ui->dspin_arrow_5->value();
        coefs.values[5]=ui->dspin_arrow_6->value();
        if(isAdjustable)emit arrowCoefficients(coefs);
    });
    connect(ui->dspin_arrow_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_arrow_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_arrow_3->value();
        coefs.values[3]=ui->dspin_arrow_4->value();
        coefs.values[4]=ui->dspin_arrow_5->value();
        coefs.values[5]=ui->dspin_arrow_6->value();
        if(isAdjustable)emit arrowCoefficients(coefs);
    });
    connect(ui->dspin_arrow_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_arrow_1->value();
        coefs.values[1]=ui->dspin_arrow_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_arrow_4->value();
        coefs.values[4]=ui->dspin_arrow_5->value();
        coefs.values[5]=ui->dspin_arrow_6->value();
        if(isAdjustable)emit arrowCoefficients(coefs);
    });
    connect(ui->dspin_arrow_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_arrow_1->value();
        coefs.values[1]=ui->dspin_arrow_2->value();
        coefs.values[2]=ui->dspin_arrow_3->value();
        coefs.values[3]=value;
        coefs.values[4]=ui->dspin_arrow_5->value();
        coefs.values[5]=ui->dspin_arrow_6->value();
        if(isAdjustable)emit arrowCoefficients(coefs);
    });
    connect(ui->dspin_arrow_5,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_arrow_1->value();
        coefs.values[1]=ui->dspin_arrow_2->value();
        coefs.values[2]=ui->dspin_arrow_3->value();
        coefs.values[3]=ui->dspin_arrow_4->value();
        coefs.values[4]=value;
        coefs.values[5]=ui->dspin_arrow_6->value();
        if(isAdjustable)emit arrowCoefficients(coefs);
    });
    connect(ui->dspin_arrow_6,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(6);
        coefs.values[0]=ui->dspin_arrow_1->value();
        coefs.values[1]=ui->dspin_arrow_2->value();
        coefs.values[2]=ui->dspin_arrow_3->value();
        coefs.values[3]=ui->dspin_arrow_4->value();
        coefs.values[4]=ui->dspin_arrow_5->value();
        coefs.values[5]=value;
        if(isAdjustable)emit arrowCoefficients(coefs);
    });

    connect(this,&Factory::arrowCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Arrow)){
            model.type=Arrow;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });


    //text3D
    connect(ui->dspin_text3D_1,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=value;
        coefs.values[1]=ui->dspin_text3D_2->value();
        coefs.values[2]=ui->dspin_text3D_3->value();
        coefs.values[3]=ui->dspin_text3D_4->value();
        if(isAdjustable)emit text3DCoefficients(coefs);
    });
    connect(ui->dspin_text3D_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=ui->dspin_text3D_1->value();
        coefs.values[1]=value;
        coefs.values[2]=ui->dspin_text3D_3->value();
        coefs.values[3]=ui->dspin_text3D_4->value();
        if(isAdjustable)emit text3DCoefficients(coefs);
    });
    connect(ui->dspin_text3D_3,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=ui->dspin_text3D_1->value();
        coefs.values[1]=ui->dspin_text3D_2->value();
        coefs.values[2]=value;
        coefs.values[3]=ui->dspin_text3D_4->value();
        if(isAdjustable)emit text3DCoefficients(coefs);
    });
    connect(ui->dspin_text3D_4,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        coefs.values.resize(4);
        coefs.values[0]=ui->dspin_text3D_1->value();
        coefs.values[1]=ui->dspin_text3D_2->value();
        coefs.values[2]=ui->dspin_text3D_3->value();
        coefs.values[3]=value;
        if(isAdjustable)emit text3DCoefficients(coefs);
    });

    connect(this,&Factory::text3DCoefficients,[=](const ModelCoefs &coefs)
    {
        Model selectedModel=modelTree->getSelectedModel();
        if((selectedModel.isEmpty)|(selectedModel.type!=Text3D)){
            model.type=Text3D;
            model.id="pre-model";
            model.isEmpty=false;
            model.coefs=coefs;
            cloudView->updateModel(model);
        }else {
            model=selectedModel;
            model.id="pre-model";
            model.coefs=coefs;
            cloudView->updateModel(model);
        }
    });

    connect(modelTree,&ModelTree::selectedModelChanged,this,&Factory::changeModel);
    connect(ui->btn_delete,&QPushButton::clicked,modelTree,&ModelTree::clearSelectedModels);
}

void Factory::add()
{
    isAdjustable=false;
    if(model.isEmpty)return;
    cloudView->removeShape("pre-model");
    switch (model.type)
    {
    case 0://Plane
        model.id="Plane";
        break;
    case 1://Sphere
        model.id="Sphere";
        break;
    case 2://Line
        model.id="Line";
        break;
    case 3://Cyplinder
        model.id="Cyplinder";
        break;
    case 4://Circle
        model.id="Circle";
        break;
    case 5://Cone
        model.id="Cone";
        break;
    case 6://Cube
        model.id="Cube";
        break;
    case 7://Arrow
        model.id="Arrow";
        break;
    case 8://Text3D
        model.id="Text3D";
        break;
        //Empty
    case 9:
        break;
    }
    modelTree->addModel(model);
    isAdjustable=true;
}

void Factory::apply()
{
    Model selectedModel=modelTree->getSelectedModel();
    Index index=modelTree->getSelectedIndex();
    if(selectedModel.isEmpty){
        console->warning(tr("Please select a model!"));
        return;
    }
    if(selectedModel.type!=model.type){
        console->warning(tr("Please select the correct model!"));
        return;
    }
    cloudView->removeShape("pre-model");
    selectedModel.coefs=model.coefs;
    modelTree->updateModel(index,selectedModel);
}

void Factory::reset()
{
    cloudView->removeShape("pre-model");
    model.isEmpty=true;
}

void Factory::changeModel(const Model &selectedModel)
{
    ui->cbox_models->setCurrentIndex(selectedModel.type);
    isAdjustable=false;
    switch (selectedModel.type)
    {
    case 0://plane
        ui->dspin_plane_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_plane_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_plane_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_plane_4->setValue(selectedModel.coefs.values[3]);
        ui->dspin_plane_5->setValue(selectedModel.coefs.values[4]);
        ui->dspin_plane_6->setValue(selectedModel.coefs.values[5]);
        ui->dspin_plane_7->setValue(selectedModel.coefs.values[6]);
        break;
    case 1://sphere
        ui->dspin_sphere_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_sphere_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_sphere_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_sphere_4->setValue(selectedModel.coefs.values[3]);
        break;
    case 2://line
        ui->dspin_line_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_line_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_line_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_line_4->setValue(selectedModel.coefs.values[3]);
        ui->dspin_line_5->setValue(selectedModel.coefs.values[4]);
        ui->dspin_line_6->setValue(selectedModel.coefs.values[5]);
        break;
    case 3://cyplinder
        ui->dspin_cyplinder_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_cyplinder_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_cyplinder_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_cyplinder_4->setValue(selectedModel.coefs.values[3]);
        ui->dspin_cyplinder_5->setValue(selectedModel.coefs.values[4]);
        ui->dspin_cyplinder_6->setValue(selectedModel.coefs.values[5]);
        ui->dspin_cyplinder_7->setValue(selectedModel.coefs.values[6]);
        break;
    case 4://circle
        ui->dspin_circle_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_circle_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_circle_3->setValue(selectedModel.coefs.values[2]);
        break;
    case 5://cone
        ui->dspin_cone_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_cone_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_cone_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_cone_4->setValue(selectedModel.coefs.values[3]);
        ui->dspin_cone_5->setValue(selectedModel.coefs.values[4]);
        ui->dspin_cone_6->setValue(selectedModel.coefs.values[5]);
        ui->dspin_cone_7->setValue(selectedModel.coefs.values[6]);
        break;
    case 6://cube
        ui->dspin_cube_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_cube_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_cube_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_cube_4->setValue(selectedModel.coefs.values[3]);
        ui->dspin_cube_5->setValue(selectedModel.coefs.values[4]);
        ui->dspin_cube_6->setValue(selectedModel.coefs.values[5]);
        break;
    case 7://arrow
        ui->dspin_arrow_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_arrow_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_arrow_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_arrow_4->setValue(selectedModel.coefs.values[3]);
        ui->dspin_arrow_5->setValue(selectedModel.coefs.values[4]);
        ui->dspin_arrow_6->setValue(selectedModel.coefs.values[5]);
        break;
    case 8://text3D
        ui->dspin_text3D_1->setValue(selectedModel.coefs.values[0]);
        ui->dspin_text3D_2->setValue(selectedModel.coefs.values[1]);
        ui->dspin_text3D_3->setValue(selectedModel.coefs.values[2]);
        ui->dspin_text3D_4->setValue(selectedModel.coefs.values[3]);
        break;
    case 9://Null
        break;
    }
    isAdjustable=true;
}

void Factory::closeEvent(QCloseEvent *event)
{
    this->reset();
    connect(modelTree,&ModelTree::selectedModelChanged,this,&Factory::changeModel);
    if(modelTree->size()<=0)
        modelTree->hide();
    return QWidget::closeEvent(event);
}
