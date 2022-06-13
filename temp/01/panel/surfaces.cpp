#include "surfaces.h"
#include "ui_surfaces.h"

Surfaces::Surfaces(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Surfaces)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Surfaces::preview);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Surfaces::reset);
    Surface *surface=new Surface;
    surface->moveToThread(&thread);
    qRegisterMetaType<PolygonMesh::Ptr>("PolygonMesh::Ptr&");
    qRegisterMetaType<PolygonMesh::Ptr>("PolygonMesh::Ptr");
    qRegisterMetaType<std::string>("std::string&");
    qRegisterMetaType<std::string>("std::string");
    connect(&thread,&QThread::finished,surface,&QObject::deleteLater);
    connect(this,&Surfaces::greedyProjectionTriangulation,surface,&Surface::greedyProjectionTriangulation);
    connect(this,&Surfaces::gridProjection,surface,&Surface::gridProjection);
    connect(this,&Surfaces::poisson,surface,&Surface::poisson);
    connect(surface,&Surface::result,this,&Surfaces::result);
    thread.start();
    ui->cbox_surface->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Surfaces::~Surfaces()
{
    thread.quit();
    thread.wait();
    delete ui;
}

void Surfaces::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    connect(ui->check_Polygonline,&QCheckBox::clicked,[=](bool state){
        std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
        for(size_t i=0;i<selectedClouds.size();i++) {
            if(!cloud_view->contains(selectedClouds[i]->id+"-mesh"))continue;
            if(state) {
              cloud_view->removePolygonMesh(selectedClouds[i]->id+"-mesh");
              cloud_view->addPolylineFromPolygonMesh(meshs[i],selectedClouds[i]->id+"-mesh");
            }else {
                cloud_view->removeShape(selectedClouds[i]->id+"-mesh");
                cloud_view->addPolygonMesh(meshs[i],selectedClouds[i]->id+"-mesh");
            }
        }
    });
    connect(cloud_tree,&CloudTree::removedId,this,&Surfaces::removeMesh);
}

void Surfaces::preview()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    meshs.clear();
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(cloud_view->contains(selectedClouds[i]->id))
            cloud_tree->setCloudChecked(indexs[i],false);
        switch (ui->cbox_surface->currentIndex()) {
        case 0://GreedyProjectionTriangulation
            emit greedyProjectionTriangulation(selectedClouds[i],ui->dspin_mu->value(),ui->spin_nnn->value(),ui->dspin_radius->value(),
                                                       ui->spin_minangle->value(),ui->spin_maxangle->value(),ui->spin_maxsurface->value());
            cloud_view->showInfo("Greedy Projection Triangulation",30,"info");
            break;
        case 1://GridProjection
            emit gridProjection(selectedClouds[i],ui->dspin_leaf_size->value(),ui->spin_k->value(),ui->spin_max_binary_search_level->value());
            cloud_view->showInfo("GridProjection",30,"info");
            break;
        case 2://Poisson
            emit poisson(selectedClouds[i],ui->spin_depth->value(),ui->spin_min_depth->value(),ui->dspin_point_weight->value(),
                                 ui->dspin_scale->value(),ui->dspin_samples_per_node->value());
            cloud_view->showInfo("Poisson",30,"info");
            break;
        }
       console->showProgressBar();
    }
}

void Surfaces::reset()
{
    cloud_view->removeShape("info");
    meshs.clear();
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(!cloud_view->contains(selectedClouds[i]->id))
            cloud_tree->setCloudChecked(indexs[i],true);
        cloud_view->removePolygonMesh(selectedClouds[i]->id+"-mesh");
        cloud_view->removeShape(selectedClouds[i]->id+"-mesh");
    }
}

void Surfaces::result(const PolygonMesh::Ptr &mesh, const std::string &id,float time)
{
    console->closeProgressBar();
    if(mesh->polygons.empty()) {
        cloud_view->removePolygonMesh(id+"-mesh");
        console->info(tr("The pointcloud ")+id.c_str()+tr("has been surfaced failed,please retry !"));
        return;
    }
    if(ui->check_Polygonline->isChecked())
        cloud_view->addPolylineFromPolygonMesh(mesh,id+"-mesh");
    else
        cloud_view->addPolygonMesh(mesh,id+"-mesh");
    console->info(tr("The pointcloud ")+id.c_str()+tr(" has been surfaced,take time %1 ms").arg(time));
    meshs.push_back(mesh);
}

void Surfaces::removeMesh(const std::string &id)
{
    cloud_view->removePolygonMesh(id+"-mesh");
    cloud_view->removeShape(id+"-mesh");
}

void Surfaces::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}
