#include "pickplan.h"
#include "ui_pickplan.h"

PickPlan::PickPlan(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::PickPlan),target_cloud(new Cloud),ail_cloud(new Cloud),target_index(-1),path_index(-1),
    eye_hand_affine(Eigen::Affine3f::Identity()),is_start(false),is_auto_run(false)
{
    ui->setupUi(this);
    connect(ui->btn_set_template,&QPushButton::clicked,this,&PickPlan::setTemplate);
    connect(ui->btn_set_target,&QPushButton::clicked,this,&PickPlan::setTarget);
    connect(ui->btn_preview,&QPushButton::clicked,this,&PickPlan::preview);
    connect(ui->btn_addfinal_path,&QPushButton::clicked,this,&PickPlan::addFinalPath);
    connect(ui->tbtn_open_eyehand,&QPushButton::clicked,this,&PickPlan::openEyeHandFile);
    connect(ui->btn_add,&QPushButton::clicked,this,&PickPlan::add);
    connect(ui->btn_start,&QPushButton::clicked,this,&PickPlan::start);
    connect(ui->btn_auto,&QPushButton::clicked,this,&PickPlan::autoRun);
    connect(ui->btn_reset,&QPushButton::clicked,this,&PickPlan::reset);
    connect(ui->tbtn_open,&QToolButton::clicked,this,&PickPlan::openPath);
}

PickPlan::~PickPlan()
{
    delete ui;
}

void PickPlan::init(Console *&co, CloudView *&cv, CloudTree *&ct, ProcessTree *pt,PathTable *ptb)
{
    console=co;cloud_view=cv;cloud_tree=ct;process_tree=pt;path_table=ptb;
    ui->tree_layout->addWidget(process_tree);
    ui->table_layout->addWidget(path_table);
    process_tree->setProcessEnable(true);
    process_tree->setExtendedSelection(false);
    ui->check_enable->setChecked(true);
    connect(ui->btn_clear,&QPushButton::clicked,path_table,&PathTable::clearSelectCommand);
    connect(process_tree,&ProcessTree::lastMoveDoneBeforeCapture,this,&PickPlan::preview);
    //connect(process_tree,&ProcessTree::lastMoveDoneAfterCapture,this,&PickPlan::start);
    connect(ui->check_enable,&QCheckBox::clicked,[=](bool checked)
    {
        if(checked)
        {
            process_tree->setEnabled(true);
            process_tree->setProcessEnable(true);
        }else {
            process_tree->setEnabled(false);
            process_tree->setProcessEnable(false);
        }
    });
    connect(ui->check_user,&QCheckBox::clicked,[=](bool checked)
    {
        if(checked)
        {
            ui->lineEdit_path->setEnabled(true);
            ui->tbtn_open->setEnabled(true);
        }else {
            ui->lineEdit_path->setEnabled(false);
            ui->tbtn_open->setEnabled(false);
        }
    });
}

void PickPlan::setTemplate()
{
    std::vector<Cloud::Ptr> all_clouds=cloud_tree->getAllClouds();
    for(auto &i:template_cloud)
        cloud_view->resetCloudColor(i,i->id);
    Template *template_dialog=new Template(this);
    template_dialog->setTemplate(all_clouds);
    template_dialog->setExtendedSelection(true);
    connect(template_dialog,&Template::getTemplate,[=](const std::vector<int> &index){
        template_cloud.clear();
        for(size_t i=0;i<index.size();i++) {
            Cloud::Ptr cloud=all_clouds[index[i]];
            cloud_view->removeShape(cloud->box_id);
            cloud_view->setCloudColor(cloud,cloud->id,0,255,0);
            template_cloud.push_back(cloud);
            console->info(tr("Select the cloud ")+cloud->id.c_str()+tr(" as template cloud"));
        }
        cloud_view->showInfo("Template Cloud : Green Cloud ",30,"info");
    });
    if(template_dialog->exec()==QDialog::Rejected) {
        console->warning(tr("Select the template cloud canceled!"));
        return;
    }
}

void PickPlan::setTarget()
{
    std::vector<Cloud::Ptr> all_clouds=cloud_tree->getAllClouds();
    if(!target_cloud->empty())
        cloud_view->resetCloudColor(target_cloud,target_cloud->id);
    Template *template_dialog=new Template(this);
    template_dialog->setTemplateWithDevice(all_clouds);
    template_dialog->setExtendedSelection(false);
    connect(template_dialog,&Template::getIndex,[=](int index){
        target_index=index;
        if(target_index==0) {
            connect(process_tree,&ProcessTree::captureCloud,this,&PickPlan::processCloud);
            console->info(tr("Select the capture cloud of device zhisensor as target cloud"));
            cloud_view->showInfo("Target Cloud : Capture Cloud ",50,"info1");
        }
        else {
            disconnect(process_tree,&ProcessTree::captureCloud,this,&PickPlan::processCloud);
            target_cloud=all_clouds[target_index-1];
            cloud_view->removeShape(target_cloud->box_id);
            cloud_view->setCloudColor(target_cloud,target_cloud->id,255,0,0);
            console->info(tr("Select the cloud ")+target_cloud->id.c_str()+tr(" as target cloud"));
            cloud_view->showInfo("Target Cloud : Red Cloud ",50,"info1");
        }
    });
    if(template_dialog->exec()==QDialog::Rejected) {
        console->warning(tr("Select the target cloud canceled!"));
        return;
    }
}

void PickPlan::addFinalPath()
{
    if(template_cloud.empty()) {
        console->warning(tr("Please select some template clouds !"));
        return;
    }
    if(template_cloud.front()->path_points.size()<=0) {
        console->warning(tr("Please set path point for template cloud !"));
        return;
    }
    for (size_t i=0;i<template_cloud.size();i++)
    {
        for(size_t j=i+1;j<template_cloud.size();j++)
        {
            if(template_cloud[i]->path_points.size()!=template_cloud[j]->path_points.size())
            {
                console->warning(tr("Please ensure the path points num of template clouds !"));
                return;
            }
        }
    }
    Template *template_dialog=new Template(this);
    template_dialog->setAllPath(template_cloud.front()->path_points.size());
    template_dialog->setExtendedSelection(false);
    connect(template_dialog,&Template::getPath,[=](int index,float speed){
        if(ui->lineEdit_eyehand->text().isEmpty()||eye_hand_affine.matrix()==Eigen::Matrix4f::Identity())
        {
            console->warning(tr("Please load robot eye hand file first!"));
            return;
        }
        if(speed==0)
        {
            console->warning(tr("Please set correct speed of path!"));
            return;
        }
        if(template_cloud.front()->is_tool_path)
        {
            Eigen::Affine3f path(template_cloud.front()->path_points[index]);
            int row=path_table->addPath(index,path,speed);
            final_path_index.push_back(row);
        }
        else
        {
            Eigen::Affine3f path(eye_hand_affine.matrix()*(template_cloud.front()->path_points[index]));
            int row=path_table->addPath(index,path,speed);
            final_path_index.push_back(row);
        }
    });
    if(template_dialog->exec()==QDialog::Rejected) {
        console->warning(tr("Add Final path canceled!"));
        return;
    }
}

void PickPlan::preview()
{
    if(template_cloud.empty()||(target_cloud->empty()&&target_index==-1)) {
        console->warning(tr("Please select some template clouds or a target cloud!"));
        return;
    }
    if(target_index==0)
        emit process_tree->startCapture();
    else {
        Cloud::Ptr copy_target_cloud=target_cloud->makeShared();
        this->processCloud(copy_target_cloud);
    }
}

void PickPlan::reset()
{
    cloud_view->removeCloud("reg");
    cloud_view->removeShape("info");
    cloud_view->removeShape("info1");
    cloud_view->removeShape("info2");
    cloud_view->removeShape("info3");
    ail_cloud->clear();
    if(!target_cloud->empty())
        cloud_view->resetCloudColor(target_cloud,target_cloud->id);
    target_cloud.reset(new Cloud);
    for(auto &i:template_cloud)
        cloud_view->resetCloudColor(i,i->id);
    template_cloud.clear();
}

void PickPlan::openEyeHandFile()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open eye hand file"));
    if(path.isEmpty())return;
    QFile file(path);
    if (!file.open(QFile::ReadOnly))
    {
        console->error(tr("Load robot eye hand file :")+path+tr(" failed!"));
        return;
    }
    QTextStream in(&file);
    QString text=in.readAll().remove("cam2base:").remove("cam2tool:");
    file.close();
    bool success=false;
    eye_hand_affine=Tool::AffineFromQString(text,success);
    if(!success) {
        console->error(tr("Eye hand file ")+path+tr(" format is wrong"));
        return;
    }
    console->info(tr("Load robot eye hand file :")+path+tr(" successfully!"));
    ui->lineEdit_eyehand->setText(path);
    //    std::cout<<eye_hand_affine.matrix()*template_cloud.front()->path_points.front()<<std::endl;
}

void PickPlan::openPath()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open eye hand file"));
    if(path.isEmpty())return;
    QFile file(path);
    if (!file.open(QFile::ReadOnly))
    {
        console->error(tr("Load robot path file :")+path+tr(" failed!"));
        return;
    }
    QTextStream in(&file);
    QStringList textlist = in.readAll().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
    if (textlist.size() != 6){
        console->error(tr("Path file format is wrong"));
        return;
    }
    file.close();
    Eigen::Affine3f pos=Tool::AffineFromXYZEuler(textlist[0].toFloat(),textlist[1].toFloat(),textlist[2].toFloat(),
            textlist[3].toFloat(),textlist[4].toFloat(),textlist[5].toFloat());
    path_index=path_table->addPath(0,pos,2);
    console->error(tr("Path file add sucessfully"));
    ui->lineEdit_path->setText(path);
}

void PickPlan::add()
{
    Template *template_dialog=new Template(this);
    template_dialog->setCommand();
    template_dialog->setExtendedSelection(false);
    connect(template_dialog,&Template::getIndex,[=](int index){
        switch (index) {
        case 0://movej
            emit process_tree->moveJ();
            break;
        case 1://movep
            emit process_tree->moveP();
            break;
        case 2://movel
            emit process_tree->moveL();
            break;
        case 3://io
            emit process_tree->ioWrite();
            break;
        case 4://capture
            path_table->addCapture();
            break;
        }
    });
    if(template_dialog->exec()==QDialog::Rejected) {
        console->warning(tr("Add Command canceled!"));
        return;
    }
}

void PickPlan::start()
{
    if(!is_start){
        this->restart();
        console->info(tr("Start pick once"));
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        is_start=true;
    } else {
        console->info(tr("Stop pick once"));
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        is_start=false;
    }
}

void PickPlan::autoRun()
{
    //connect(process_tree,&ProcessTree::lastMoveDoneAfterCapture,this,&PickPlan::start);
    if(is_start)
        this->start();
    if(!is_auto_run){
        connect(process_tree,&ProcessTree::lastMoveDoneAfterCapture,this,&PickPlan::restart);
        this->restart();
        console->info(tr("Start pick auto"));
        ui->btn_auto->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        is_auto_run=true;
    } else {
        disconnect(process_tree,&ProcessTree::lastMoveDoneAfterCapture,this,&PickPlan::restart);
        console->info(tr("Stop pick auto"));
        ui->btn_auto->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        is_auto_run=false;
    }
}

void PickPlan::processCloud(Cloud::Ptr &target_cloud)
{
    if(!process_tree->enable()) {
        console->warning(tr("Please enable the process tree!"));
        return;
    }
    std::vector<Process::Ptr> all_process=process_tree->getAllProcess();
    pcl::console::TicToc time;
    time.tic();
    for(auto &i:all_process) {
        switch(i->type)
        {
        case process_filter:
            switch(int(i->value[0])){
            case 0://PassThrough
                target_cloud->swap(*(Filter::PassThrough(target_cloud,i->value[2],i->value[3],i->value[4],i->value[1])));
                break;
            case 1://VoxelGrid
                target_cloud->swap(*(Filter::VoxelGrid(target_cloud,i->value[2],i->value[3],i->value[4],i->value[1])));
                break;
            case 2://PlaneRemoval
                target_cloud->swap(*(Filter::PlaneRemoval(target_cloud,i->value[2],i->value[3],i->value[1])));
                break;
            case 3://StatisticalOutlierRemoval
                target_cloud->swap(*(Filter::StatisticalOutlierRemoval(target_cloud,i->value[2],i->value[3],i->value[1])));
                break;
            case 4://RadiusOutlierRemoval
                target_cloud->swap(*(Filter::RadiusOutlierRemoval(target_cloud,i->value[2],i->value[3],i->value[1])));
                break;
            }
            cloud_view->updateCloud(target_cloud,target_cloud->id);
            break;
        case process_registration:
            switch (int(i->value[0]))
            {
            case 0://SampleConsensusInitialAlignment
                final_result=Registration::SampleConsensusInitialAlignment(target_cloud,template_cloud,i->value[1],i->value[2],i->value[3],i->value[4],i->value[5],
                        i->value[6],i->value[7]);
                break;
            case 1://SampleConsensusPrerejective
                final_result=Registration::SampleConsensusPrerejective(target_cloud,template_cloud,i->value[1],i->value[2],i->value[3],i->value[4],i->value[5],
                        i->value[6],i->value[7],i->value[8]);
                break;
            }
            if(final_result.index==-1) {
                cloud_view->removeCloud("reg");
                console->warning(tr("Registration has not converged.Please retry!"));
                return;
            } else {
                pcl::transformPointCloud(*(template_cloud[final_result.index]),*ail_cloud,final_result.final_transformation);
                cloud_view->addCloud(ail_cloud,"reg");
                cloud_view->setCloudColor(ail_cloud,"reg",0,0,255);
                if(ui->check_show_box->isChecked())
                {
                    ail_cloud->computerBox();
                    cloud_view->addCube(ail_cloud->box,ail_cloud->box_id);
                } else {
                    cloud_view->removeShape(ail_cloud->box_id);
                }
                cloud_view->showInfo("Fitness Score: "+std::to_string(final_result.fitness_score),90,"info3");
            }
            if(!ui->check_user->isChecked()){
                path_points.clear();
                for(size_t i=0;i<template_cloud[final_result.index]->path_points.size();i++)
                {
                    Eigen::Matrix4f path_point=final_result.final_transformation*template_cloud[final_result.index]->path_points[i];
                    if(template_cloud[final_result.index]->is_tool_path){
                        Eigen::Affine3f path_point_aff_(path_point);
                        path_points.push_back(path_point_aff_);
                    } else {
                        if(ui->check_show_path->isChecked())
                        {
                            Eigen::Affine3f path_point_aff(path_point);
                            Eigen::Vector3f xyz=path_point.topRightCorner(3, 1);
                            cloud_view->addCoord(60,path_point_aff,"path_"+std::to_string(i));
                            cloud_view->addText3D("path_txt"+std::to_string(i),PointXYZRGBN(xyz[0],xyz[1],xyz[2],0,0,0),10,0,255,255,"path_txt"+std::to_string(i));
                        } else {
                            cloud_view->removeCoord("path_"+std::to_string(i));
                            cloud_view->removeShape("path_txt"+std::to_string(i));
                        }
                        Eigen::Affine3f path_point_aff_(eye_hand_affine.matrix()*path_point);
                        path_points.push_back(path_point_aff_);
                    }
                }
                for(size_t j=0;j<final_path_index.size();j++)
                {
                    int path_num=path_table->pathPointNum(final_path_index[j]);
                    path_table->updatePath(final_path_index[j],path_points[path_num]);
                }
            }else {
                QFile file(ui->lineEdit_path->text());
                if (!file.open(QFile::ReadOnly))
                {
                    console->error(tr("Load robot path file failed!"));
                    return;
                }
                QTextStream in(&file);
                QStringList textlist = in.readAll().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
                if (textlist.size() != 6){
                    console->error(tr("Path file format is wrong"));
                    return;
                }
                file.close();
                Eigen::Affine3f pos=Tool::AffineFromXYZEuler(textlist[0].toFloat(),textlist[1].toFloat(),textlist[2].toFloat(),
                        textlist[3].toFloat(),textlist[4].toFloat(),textlist[5].toFloat());
                path_table->updatePath(path_index,pos);
            }
            break;
        case process_transformation:
            break;
        case process_coordinate:
            break;
        case process_empty:
            break;
        case process_device:
            break;
        case process_color:
            break;
        }
    }
    console->info(tr("All processes has done successfully,take time: %1 ms").arg(time.toc()));
    if(is_start){
        std::vector<Position> pos1=path_table->getMoveDatas(false);
        std::vector<IOWrite> io1=path_table->getIODatas(false);
        emit process_tree->startMove(pos1,io1,false);
    }
    if(is_auto_run){
        std::vector<Position> pos1=path_table->getMoveDatas(false);
        std::vector<IOWrite> io1=path_table->getIODatas(false);
        emit process_tree->startMove(pos1,io1,false);
    }
}

void PickPlan::restart()
{
    std::vector<Position> pos1=path_table->getMoveDatas(true);
    std::vector<IOWrite> io1=path_table->getIODatas(true);
    emit process_tree->startMove(pos1,io1,true);
}

void PickPlan::closeEvent(QCloseEvent *event)
{
    ui->tree_layout->removeWidget(process_tree);
    process_tree->setParent(nullptr);
    process_tree->setProcessEnable(false);
    //process_tree->clearAllProcesses();
    ui->table_layout->removeWidget(path_table);
    path_table->setParent(nullptr);
    return QDockWidget::closeEvent(event);
}
