#ifndef SAMPLINS_H
#define SAMPLINGS_H

#include <QDialog>
#include "common/cloudtree.h"
#include "modules/sampling.h"

namespace Ui {
class Samplings;
}

class Samplings : public QDialog
{
    Q_OBJECT

public:
    explicit Samplings(QWidget *parent = nullptr);
    ~Samplings();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void preview();
    void add();
    void apply();
    void reset();

signals:
    void uniformSampling(const Cloud::Ptr& cloud,float);
    void downSampling(const Cloud::Ptr& cloud,float);
    void randomSampling(const Cloud::Ptr& cloud,int,int);
    void samplingSurfaceNormal(const Cloud::Ptr& cloud,int,int,float);
    void normalSpaceSampling(const Cloud::Ptr& cloud,int,int,float);

public slots:
    void result(const Cloud::Ptr &cloud,float time);
    void removeCloud(const std::string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Samplings *ui;

    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    QThread thread;
    std::vector<Cloud::Ptr> sampling_clouds;
};

#endif // SAMPLINGS_H
