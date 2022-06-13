#ifndef SURFACES_H
#define SURFACES_H

#include <QDockWidget>
#include <QThread>
#include "common/cloudtree.h"
#include "modules/surface.h"

namespace Ui {
class Surfaces;
}

class Surfaces : public QDockWidget
{
    Q_OBJECT

public:
    explicit Surfaces(QWidget *parent = nullptr);
    ~Surfaces();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void preview();
    void reset();

signals:
    void greedyProjectionTriangulation(const Cloud::Ptr &cloud, double mu,int nnn,double radius,int min,int max,int surface);
    void gridProjection(const Cloud::Ptr &cloud, double resolution,int k,int max_binary_search_level);
    void poisson(const Cloud::Ptr &cloud,int depth, int min_depth,float point_weight,float scale,float samples_per_node);

public slots:
    void result(const PolygonMesh::Ptr &mesh,const std::string &id,float time);
    void removeMesh(const std::string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Surfaces *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    QThread thread;
    std::vector<PolygonMesh::Ptr> meshs;
};

#endif // SURFACES_H
