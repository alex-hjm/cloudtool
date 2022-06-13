#ifndef FILTERS_H
#define FILTERS_H

#include <QDockWidget>
#include <QThread>
#include "common/cloudtree.h"
#include "modules/filter.h"

namespace Ui {
class Filters;
}

class Filters : public QDockWidget
{
    Q_OBJECT

public:
    explicit Filters(QWidget *parent = nullptr);
    ~Filters();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void preview();
    void add();
    void apply();
    void reset();
signals:
    void passThrough(const Cloud::Ptr& cloud,const std::string &field_name,float limit_min,float limit_max,bool negative);
    void voxelGrid(const Cloud::Ptr& cloud,float lx, float ly, float lz,bool negative);
    void approximateVoxelGrid (const Cloud::Ptr& cloud,float lx, float ly, float lz);
    void statisticalOutlierRemoval (const Cloud::Ptr& cloud,int nr_k,double stddev_mult,bool negative);
    void radiusOutlierRemoval (const Cloud::Ptr& cloud,double radius,int min_pts,bool negative);
    void movingLeastSquares (const Cloud::Ptr& cloud,bool computer_normals,int polynomial_order,float radius);
public slots:
    void result(const Cloud::Ptr &cloud,float time);
    void getRangeFromCloud(int index);
    void removeCloud(const std::string &id);
protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Filters *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    QThread thread;
    std::vector<Cloud::Ptr> filtered_clouds;
};

#endif // FILTERS_H
