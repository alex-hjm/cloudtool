#ifndef CLOUDTREE_H
#define CLOUDTREE_H

#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>
#include <QSpinBox>
#include <QCheckBox>

#include "customtree.h"
#include "cloudview.h"
#include "console.h"

#include "src/common/cloud.h"
#include "src/modules/fileio.h"

class CloudTree : public CustomTree
{
    Q_OBJECT
public:
    explicit CloudTree(QWidget *parent = nullptr);

    Console *console;
    CloudView *cloudView;
    QTableWidget *cloudTable;

    bool loadCloud(const QString &path);
    void addClouds();
    void addSamlpeCloud();
    void insertCloud(int row,Cloud &cloud,const bool&selected);
    void updateCloud(const Index &index,Cloud &updatecloud);
    void updateSelectedClouds(std::vector<Cloud> &updateclouds);
    void clearCloud(const Index &index);
    void clearSelectedClouds();
    void clearAllClouds();
    void setCloudChecked(const Index &index,bool checked);
    void setSelectedCloudsChecked(bool checked);
    void setCloudSelected(const Index &index,bool selected);
    void setSelectedCloudsSelected(bool selected);
    void saveSelectedClouds();
    void mergeSelectedClouds();
    void cloneSelectedClouds();
    void renameSelectedClouds();

    inline size_t size(){return cloudVec.size();}
    Cloud mergeClouds(std::vector<Cloud> &clouds);
    Cloud getCloud(const Index &index);
    Cloud getSelectedCloud();
    std::vector<Cloud> getSelectedClouds();
    std::vector<Cloud> getAllClouds();
    void setAcceptDrops(const bool &enable);
    void setExtendedSelection(const bool &enable);

signals:
    void removedId(const string &id);

public slots:
    void updateProperty();
    void updateScreen(QTreeWidgetItem *, int);

protected:
    void mousePressEvent(QMouseEvent *event);

private:

    FileIO fileio;
    QFileInfo lastFileInfo;
    std::vector<std::vector<Cloud>> cloudVec;
};


#endif // CLOUDTREE_H
