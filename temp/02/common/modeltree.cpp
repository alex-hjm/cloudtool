#include "modeltree.h"

ModelTree::ModelTree(QWidget *parent) : CustomTree(parent)
{
    connect(this,&ModelTree::itemSelectionChanged,this,&ModelTree::updateProperty);
    connect(this,SIGNAL(itemClicked(QTreeWidgetItem *, int)),this,SLOT(updateScreen(QTreeWidgetItem *, int)));
    repersentation_list <<  tr("points")<< tr("wireframe") <<tr("surface");
    shading_list << tr("flat") << tr("gouraud") << tr("phong");
}

void ModelTree::addModel(Model &model)
{
    if(modelVec.size()<=0)
        this->insertModel(-1,model,false);
    else
        this->insertModel(0,model,false);
}

void ModelTree::insertModel(int row, Model &model,const bool&selected)
{
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/child2category.svg");
    QString path="Custom Model";
    if(cloudView->contains(model.id)) {
        int k= QMessageBox::information(this,"",tr("The same name exists"),tr("rename"),tr("cancel"));
        if(k==0) {
            bool ok = false;
            QString res = QInputDialog::getText(this, tr("Rename"), tr("Rename"),
                                                QLineEdit::Normal, QString::fromStdString(model.id),&ok, Qt::WindowFlags());
            if (ok) {
                if(res==model.id.c_str()) {
                    console->warning(tr("The id ")+ res+tr(" already exists! Please rename a different id and retry."));
                    return ;
                }
                else{
                    if(cloudView->contains(res.toStdString())) {
                        console->warning(tr("The id ")+ res+tr(" already exists! Please rename a different id and retry."));
                        return ;
                    }
                    model.id=res.toStdString();
                    this->addItem<Model>(row,model,path,model.id.c_str(),parentIcon,childIcon,selected,modelVec);
                    cloudView->updateModel(model);
                }
            }
        }
    }
    else {
        this->addItem<Model>(row,model,path,model.id.c_str(),parentIcon,childIcon,selected,modelVec);
        cloudView->updateModel(model);
    }
}

void ModelTree::updateModel(const Index &index,Model &updatemodel)
{
    this->updateItem<Model>(index,updatemodel,modelVec);
    cloudView->updateModel(updatemodel);
}

void ModelTree::updateSelectedModels(std::vector<Model> &updatemodels)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->updateModel(indexs[i],updatemodels[i]);
    }
}

void ModelTree::clearModel(const Index &index)
{
    Model model=this->getModel(index);
    cloudView->removeShape(model.id);
    this->removeItem<Model>(index,modelVec);
}

void ModelTree::clearSelectedModels()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    indexs=this->getSortedIndexs(descending,indexs);
    for(int i=0;i<indexs.size();i++) {
        this->clearModel(indexs[i]);
    }
}

void ModelTree::clearAllModels()
{
    this->modelVec.clear();
    this->clear();
    cloudView->removeAllShapes();
}

void ModelTree::setModelChecked(const Index &index, bool checked)
{
    this->setItemChecked(index,checked);
    Model model=this->getModel(index);
    if(checked) {
        if(!cloudView->contains(model.id))
            cloudView->updateModel(model);
    }
    else {
        cloudView->removeShape(model.id);
    }
}

void ModelTree::setSelectedModelsChecked(bool checked)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->setModelChecked(indexs[i],checked);
    }
}

void ModelTree::setModelSelected(const Index &index, bool selected)
{
    this->setItemSelected(index,selected);
    if(selected) {
        this->setModelChecked(index,true);
    }
}

void ModelTree::setSelectedModelsSelected(bool selected)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(int i=0;i<indexs.size();i++) {
        this->setModelSelected(indexs[i],selected);
    }
}

void ModelTree::cloneSelectedClouds()
{
    std::vector<Model> selectedModels=this->getSelectedModels();
    std::vector<Index> indexs=this->getSelectedIndexs();
    QIcon parentIcon(":/icon/resource/icon/document-open.svg");
    QIcon childIcon(":/icon/resource/icon/child2category.svg");
    for(size_t i=0;i<selectedModels.size();i++) {
        Model c_model=selectedModels[i];
        c_model.points.reset(new CloudXYZRGBN);
        *c_model.points=*selectedModels[i].points;
        c_model.id="clone-"+c_model.id;
        this->addItem<Model>(indexs[i].row,c_model,"",c_model.id.c_str(),parentIcon,childIcon,false,modelVec);
        cloudView->updateModel(c_model);
    }
}

Model ModelTree::getModel(const Index &index)
{
    return this->getData<Model>(index,modelVec);
}

Model ModelTree::getSelectedModel()
{
    Model selectedModel;
    if(this->getSelectedModels().size()<=0)
        return selectedModel;
    else
        return this->getSelectedModels()[0];
}

std::vector<Model> ModelTree::getSelectedModels()
{
    return this->getSelectedDatas<Model>(modelVec);
}

std::vector<Model> ModelTree::getAllModels()
{
    return this->getAllDatas(modelVec);
}

void ModelTree::setExtendedSelection(const bool &enable)
{
    if(enable)
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        this->setSelectionMode(QAbstractItemView::SingleSelection);
}

void ModelTree::updateProperty()
{
    QString type;
    auto items = this->selectedItems();
    if(items.size()<=0) {
        type="";
        modelTable->removeCellWidget(1,1);
        modelTable->removeCellWidget(2,1);
        modelTable->removeCellWidget(3,1);
        modelTable->removeCellWidget(4,1);
        modelTable->removeCellWidget(5,1);
        modelTable->removeCellWidget(6,1);
        cloudView->showCloudId("");
    } else{
        Model selectedModel=this->getSelectedModel();
        if(selectedModel.isEmpty)return;
        if(selectedModel.type==Mesh)return;
        emit selectedModelChanged(selectedModel);
        //size
        QSpinBox *pointSize=new QSpinBox();
        pointSize->setMaximumHeight(24);
        pointSize->setRange(1,99);
        pointSize->setValue(selectedModel.pointSize);
        connect(pointSize,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value) {
            Model selectedModel=this->getSelectedModel();
            Index index=this->getSelectedIndex();
            selectedModel.pointSize=value;
            this->updateModel(index,selectedModel);
        });
        modelTable->setCellWidget(1,1,pointSize);

        //opacity
        QDoubleSpinBox *opacity=new QDoubleSpinBox();
        opacity->setMaximumHeight(24);
        opacity->setSingleStep(0.1);
        opacity->setRange(0,1);
        opacity->setValue(selectedModel.opacity);
        connect(opacity,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
            Model selectedModel=this->getSelectedModel();
            Index index=this->getSelectedIndex();
            selectedModel.opacity=value;
            this->updateModel(index,selectedModel);
        });
        modelTable->setCellWidget(2,1,opacity);

        //linewidth
        QSpinBox *linewidth=new QSpinBox();
        linewidth->setMaximumHeight(24);
        linewidth->setRange(1,99);
        linewidth->setValue(selectedModel.linewidth);
        connect(linewidth,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value) {
            Model selectedModel=this->getSelectedModel();
            Index index=this->getSelectedIndex();
            selectedModel.linewidth=value;
            this->updateModel(index,selectedModel);
        });
        modelTable->setCellWidget(3,1,linewidth);

        //repersentation
        QComboBox *repersentation=new QComboBox();
        repersentation->addItems(repersentation_list);
        repersentation->setMaximumHeight(24);
        repersentation->setCurrentIndex(selectedModel.repersentation);
        if(selectedModel.repersentation==0)
            linewidth->setEnabled(false);
        else if(selectedModel.repersentation==1)
            pointSize->setEnabled(false);
        else {
            linewidth->setEnabled(false);
            pointSize->setEnabled(false);
        }
        connect(repersentation,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int currentindex) {
            Model selectedModel=this->getSelectedModel();
            Index index=this->getSelectedIndex();
            if(currentindex==0) {
                linewidth->setEnabled(false);
                pointSize->setEnabled(true);
            }
            else if(currentindex==1) {
                pointSize->setEnabled(false);
                linewidth->setEnabled(true);
            }
            else{
                linewidth->setEnabled(false);
                pointSize->setEnabled(false);
            }
            selectedModel.repersentation=currentindex;
            this->updateModel(index,selectedModel);
        });
        modelTable->setCellWidget(4,1,repersentation);

        //fontSize
        QSpinBox *fontSize=new QSpinBox();
        fontSize->setMaximumHeight(24);
        fontSize->setRange(1,99);
        fontSize->setValue(selectedModel.fontSize);
        if(selectedModel.text=="")
            fontSize->setEnabled(false);
        connect(fontSize,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value) {
            Model selectedModel=this->getSelectedModel();
            Index index=this->getSelectedIndex();
            selectedModel.fontSize=value;
            this->updateModel(index,selectedModel);
        });
        modelTable->setCellWidget(5,1,fontSize);

        //shading
        QComboBox *shading=new QComboBox();
        shading->addItems(shading_list);
        shading->setMaximumHeight(24);
        shading->setCurrentIndex(selectedModel.shading);
        connect(shading,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int currentindex) {
            Model selectedModel=this->getSelectedModel();
            Index index=this->getSelectedIndex();
            selectedModel.shading=currentindex;
            this->updateModel(index,selectedModel);
        });
        modelTable->setCellWidget(6,1,shading);

        //update
        type=this->getModelType(selectedModel.type);
    }
    modelTable->setItem(0, 1, new QTableWidgetItem(type));

}

void ModelTree::updateScreen(QTreeWidgetItem *item, int)
{
    Qt::CheckState CheckState=item->checkState(0);
    std::vector<Index> indexs=this->getClickedIndexs(item);
    if(indexs.size()>1) {//parent
        for(int i=0;i<indexs.size();i++) {
            if(CheckState==Qt::Checked) {
                this->setModelChecked(indexs[i],true);
            }
            else {
                this->setModelChecked(indexs[i],false);
            }
        }
    } else {//child
        if(CheckState==Qt::Unchecked) {
            this->setModelChecked(indexs[0],false);
        }
        else {
            this->setModelChecked(indexs[0],true);
        }
    }
}

QString ModelTree::getModelType(const model_type &type)
{
    switch (type) {
    case 0:return tr("plane");
    case 1:return tr("sphere");
    case 2:return tr("line");
    case 3:return tr("cyplinder");
    case 4:return tr("circle");
    case 5:return tr("cone");
    case 6:return tr("cube");
    case 7:return tr("arrow");
    case 8:return tr("text3D");
    case 9:return tr("Mesh");
    case 10:return tr("polygon");
    case 11:return tr("polyline");
    }
}
