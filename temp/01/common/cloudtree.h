#ifndef CLOUDTREE_H
#define CLOUDTREE_H
#include <QTableWidget>
#include <QFileDialog>
#include <QMenu>
#include <QMessageBox>
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QInputDialog>
#include <QThread>
#include <QDebug>
#include "console.h"
#include "customtree.h"
#include "cloudview.h"
#include "modules/fileio.h"

class CloudTree : public CustomTree
{
    Q_OBJECT
public:
    explicit CloudTree(QWidget *parent = nullptr);
    ~CloudTree();

    void init(Console* &,CloudView* &,QTableWidget* &);
    void addClouds();
    void addSamlpe();
    bool insertCloud(int row,const Cloud::Ptr &cloud,bool selected);
    void clearCloud(const Index &index);
    void clearSelectedClouds();
    void clearAllClouds();
    void setCloudChecked(const Index &index,bool checked);
    void setSelectedCloudsChecked(bool checked);
    void setCloudSelected(const Index &index,bool selected);
    void saveSelectedClouds();
    void mergeSelectedClouds();
    void cloneSelectedClouds();
    void renameSelectedClouds();

    Cloud::Ptr getCloud(const Index &index);
    Cloud::Ptr getSelectedCloud();
    std::vector<Cloud::Ptr> getSelectedClouds();
    std::vector<Cloud::Ptr> getAllClouds();

    void setAcceptDrops(bool enable);
    void setExtendedSelection(bool enable);

signals:
    void removedId(const std::string& id);
    void loadPointCloud(const QString& path);
    void savePointCloud(const Cloud::Ptr &cloud,const QString& path,bool isBinary);

public slots:
    void loadCloudResult(bool success,const Cloud::Ptr &cloud,float time);
    void saveCloudResult(bool success,const QString& path,float time);
    void updatePropertiesTable();
    void updateScreen(QTreeWidgetItem *, int);

protected:
    void mousePressEvent(QMouseEvent *event);

private:
    Console *console;
    CloudView* cloud_view;
    QTableWidget *cloud_table;
    QThread thread;
    std::vector<std::vector<Cloud::Ptr>> cloud_vec;
};


#endif // CLOUDTREE_H
