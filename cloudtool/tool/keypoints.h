#ifndef KEYPOINTS_H
#define KEYPOINTS_H

#include <QThread>
#include <set>

#include "pca/customdock.h"
#include "pca/keypoints.h"

namespace Ui {
class KeyPoints;
}

class KeyPoints : public pca::CustomDock {
  Q_OBJECT

 public:
  explicit KeyPoints(QWidget *parent = nullptr);
  ~KeyPoints();
  virtual void init();
  void preview();
  void add();
  void apply();
  virtual void reset();

  static std::unordered_map<QString, pca::Cloud::Ptr> keypoints_map;

 signals:
  void HarrisKeypoint3D(int response_method, float radius, float threshold,
                        bool non_maxima, bool do_refine);
  void ISSKeypoint3D(double salient_radius, double non_max_radius,
                     double normal_radius, double border_radius,
                     double gamma_21, double gamma_32, int min_neighbors,
                     float angle);
  void SIFTKeypoint(float min_scale, int nr_octaves, int nr_scales_per_octave,
                    float min_contrast);
  void TrajkovicKeypoint3D(int compute_method, int window_size,
                           float frist_threshold, float second_threshold);

 public slots:
  void keypointsResult(const pca::Cloud::Ptr &cloud, float time);

 private:
  bool checkValid(bool preview = false);

 private:
  Ui::KeyPoints *ui;
  QThread thread;
  pca::Keypoints *keypoints;
  std::vector<pca::Cloud::Ptr> selected_clouds;
  std::unordered_map<QString, pca::Cloud::Ptr> keypoints_map_tmp;
};

#endif  // KEYPOINTS_H
