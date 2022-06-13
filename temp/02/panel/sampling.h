#ifndef SAMPLING_H
#define SAMPLING_H

#include <QDialog>
#include "src/common/cloudtree.h"
#include "src/modules/samplings.h"

namespace Ui {
class Sampling;
}

class Sampling : public QDialog
{
    Q_OBJECT

public:
    explicit Sampling(QWidget *parent = nullptr);
    ~Sampling();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void preview();
    void add();
    void apply();
    void reset();

signals:
    void uniformSampling(float);
    void downSampling(float);
    void randomSampling(int,int);
    void samplingSurfaceNormal(int,int,float);
    void normalSpaceSampling(int,int,float);

public slots:
    void removeCloud(const string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Sampling *ui;
    Samplings  sam;
    std::vector<CloudXYZRGBN::Ptr> samplingClouds;
};

#endif // SAMPLING_H
