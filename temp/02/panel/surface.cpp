#include "surface.h"
#include "ui_surface.h"

Surface::Surface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Surface)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Surface::preview);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Surface::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Surface::reset);
    ui->cbox_surface->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Surface::~Surface()
{
    delete ui;
}

void Surface::init()
{
    connect(ui->check_Polygonline,&QCheckBox::clicked,[=](bool state){
        std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
        if((selectedClouds.size()>0)&&(selectedClouds.size()==meshs.size())) {
            if(state) {
                for(size_t i=0;i<selectedClouds.size();i++) {
                    cloudView->removeMesh(selectedClouds[i].id+"-mesh");
                    cloudView->updateMeshLine(meshs[i],selectedClouds[i].id+"-mesh");
                }
            }
            else {
                for(size_t i=0;i<selectedClouds.size();i++) {
                    cloudView->removeShape(selectedClouds[i].id+"-mesh");
                    cloudView->updateMesh(meshs[i],selectedClouds[i].id+"-mesh");
                }
            }

        }
        Model selectedModel=modelTree->getSelectedModel();
        if((!selectedModel.isEmpty)&&(selectedModel.type==Mesh)){
            if(state) {
                cloudView->removeMesh(selectedModel.id);
                cloudView->updateMeshLine(selectedModel.mesh,selectedModel.id);
            }
            else {
                cloudView->removeShape(selectedModel.id);
                cloudView->updateMesh(selectedModel.mesh,selectedModel.id);
            }
        }
    });
    modelTree->setExtendedSelection(false);
    connect(cloudTree,&CloudTree::removedId,this,&Surface::removeCloud);
    connect(ui->btn_delete,&QPushButton::clicked,modelTree,&ModelTree::clearSelectedModels);
}

void Surface::preview()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    int currentIndex=ui->cbox_surface->currentIndex();
    meshs.clear();
    PolygonMesh::Ptr triangle(new PolygonMesh);
    for(size_t i=0;i<selectedClouds.size();i++) {
        switch (currentIndex) {
        case 0://GreedyProjectionTriangulation
            triangle=sur.GreedyProjectionTriangulation(selectedClouds[i].cloud,ui->dspin_mu->value(),ui->spin_nnn->value(),ui->dspin_radius->value(),
                                                       ui->spin_minangle->value(),ui->spin_maxangle->value(),ui->spin_maxsurface->value());
            cloudView->showInfoText("Greedy Projection Triangulation",30,"info");
            break;
        case 1://GridProjection
            triangle=sur.GridProjection(selectedClouds[i].cloud,ui->dspin_leaf_size->value(),ui->spin_k->value(),ui->spin_max_binary_search_level->value());
            cloudView->showInfoText("GridProjection",30,"info");
            break;
        case 2://Poisson
            triangle=sur.Poisson(selectedClouds[i].cloud,ui->spin_depth->value(),ui->spin_min_depth->value(),ui->dspin_point_weight->value(),
                                 ui->dspin_scale->value(),ui->dspin_samples_per_node->value());
            cloudView->showInfoText("Poisson",30,"info");
            break;
        }
        if(triangle->polygons.size()<=0) {
            cloudView->removeMesh(selectedClouds[i].id+"-mesh");
            console->info(tr("The pointcloud ")+selectedClouds[i].id.c_str()+tr(" has been surfaced failed,please retry !"));
            return;

        }
        if(ui->check_Polygonline->isChecked())
            cloudView->updateMeshLine(triangle,selectedClouds[i].id+"-mesh");
        else
            cloudView->updateMesh(triangle,selectedClouds[i].id+"-mesh");
        console->info(tr("The pointcloud ")+selectedClouds[i].id.c_str()+tr(" has been surfaced,take time %1 ms").arg(sur.tocTime));
        meshs.push_back(triangle);
    }
}

void Surface::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(meshs.size()<=0)
        this->preview();
    cloudView->removeShape("info");
    modelTree->show();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeMesh(selectedClouds[i].id+"-mesh");
        cloudView->removeShape(selectedClouds[i].id+"-mesh");
        Model model;
        model.type=Mesh;
        model.isEmpty=false;
        model.mesh.reset(new PolygonMesh);
        *model.mesh=*meshs[i];
        model.id="mesh-"+selectedClouds[i].id;
        modelTree->addModel(model);
    }
    meshs.clear();
}

void Surface::reset()
{
    cloudView->removeShape("info");
    meshs.clear();
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeMesh(selectedClouds[i].id+"-mesh");
        cloudView->removeShape(selectedClouds[i].id+"-mesh");
    }
}

void Surface::removeCloud(const string &id)
{
    cloudView->removeMesh(id+"mesh");
    cloudView->removeShape(id+"mesh");
}

void Surface::closeEvent(QCloseEvent *event)
{
    this->reset();
    if(modelTree->size()<=0)
        modelTree->hide();
    return QWidget::closeEvent(event);
}
