/**
 * @file filters.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-18
 */
#ifndef CT_EDIT_KEYPOINTS_H
#define CT_EDIT_KEYPOINTS_H

#include "base/customdock.h"
#include "modules/keypoints.h"
#include "cloudtool/tool/rangeimage.h"

namespace Ui
{
    class KeyPoints;
}

class KeyPoints : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit KeyPoints(QWidget* parent = nullptr);
    ~KeyPoints();

    void setRangeImage(RangeImage* rangeimage)
    {
        m_rangeimage = rangeimage;
        if (rangeimage) connect(rangeimage, &RangeImage::destroyed, [=] {m_rangeimage = nullptr;});
    }
    void preview();
    void add();
    void apply();
    virtual void reset();

signals:
    void NarfKeypoint(const ct::RangeImage::Ptr range_image, float support_size);
    void HarrisKeypoint3D(int response_method, float threshold, bool non_maxima, bool do_refine);
    void ISSKeypoint3D(double resolution, double gamma_21, double gamma_32, int min_neighbors, float angle);
    void SIFTKeypoint(float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast);
    void TrajkovicKeypoint3D(int compute_method, int window_size, float frist_threshold, float second_threshold);

public slots:
    void keypointsResult(const ct::Cloud::Ptr& cloud, float time);

private:
    Ui::KeyPoints* ui;
    QThread m_thread;
    RangeImage* m_rangeimage;
    ct::Keypoints* m_keypoints;
    std::unordered_map<QString, ct::Cloud::Ptr> m_keypoints_map;
};

#endif  // KEYPOINTS_H
