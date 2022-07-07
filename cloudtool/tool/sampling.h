/**
 * @file sampling.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_SAMPLING_H
#define CT_TOOL_SAMPLING_H

#include "base/customdialog.h"
#include "modules/filters.h"
#include <QThread>

namespace Ui
{
    class Sampling;
}

class Sampling : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Sampling(QWidget* parent = nullptr);
    ~Sampling();

    void preview();
    void add();
    void apply();
    virtual void reset();

signals:
    void DownSampling(float radius);
    void UniformSampling(float radius);
    void RandomSampling(int sample, int seed);
    void ReSampling(float radius,int order);
    void SamplingSurfaceNormal(int sample, int seed, float ratio);
    void NormalSpaceSampling(int sample, int seed, int bin);

public slots:
    void samplingResult(const ct::Cloud::Ptr& cloud, float time);

private:
    Ui::Sampling* ui;
    QThread m_thread;
    ct::Filters* m_filters;
    std::map<QString, ct::Cloud::Ptr> m_sampling_map;
};

#endif // SAMPLINGS_H
