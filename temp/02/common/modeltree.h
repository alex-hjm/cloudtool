#ifndef MODELTREE_H
#define MODELTREE_H

#include <QWidget>
#include <QFileInfo>
#include <QMessageBox>
#include <QInputDialog>
#include <QSpinBox>
#include <QComboBox>

#include "customtree.h"
#include "cloudview.h"
#include "console.h"

class ModelTree: public CustomTree
{
    Q_OBJECT
public:
    explicit ModelTree(QWidget *parent = nullptr);

    Console *console;
    CloudView *cloudView;
    QTableWidget *modelTable;

    void addModel(Model &model);
    void insertModel(int row,Model &model,const bool&selected);
    void updateModel(const Index &index,Model &updatemodel);
    void updateSelectedModels(std::vector<Model> &updatemodels);
    void clearModel(const Index &index);
    void clearSelectedModels();
    void clearAllModels();
    void setModelChecked(const Index &index,bool checked);
    void setSelectedModelsChecked(bool checked);
    void setModelSelected(const Index &index,bool selected);
    void setSelectedModelsSelected(bool selected);
    void cloneSelectedClouds();

    inline size_t size(){return modelVec.size();}
    Model getModel(const Index &index);
    Model getSelectedModel();
    std::vector<Model> getSelectedModels();
    std::vector<Model> getAllModels();

    void setExtendedSelection(const bool &enable);
signals:
    void selectedModelChanged(const Model&);

public slots:
    void updateProperty();
    void updateScreen(QTreeWidgetItem *, int);

private:
    QString getModelType(const model_type &type);

//    Index index;
    QStringList repersentation_list;
    QStringList shading_list;
//    std::vector<Model> selectedModels;
//    std::vector<Model> clickedModels;
    std::vector<std::vector<Model>> modelVec;

};

#endif // MODELTREE_H
