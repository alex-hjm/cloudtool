#ifndef NORMALS_H
#define NORMALS_H

#include <QThread>

#include "pca/customdock.h"
#include "pca/features.h"

namespace Ui {
class Normals;
}

class Normals : public pca::CustomDock {
  Q_OBJECT

 public:
  explicit Normals(QWidget *parent = nullptr);
  ~Normals();
  virtual void init();
  void preview();
  void add();
  void apply();
  virtual void reset();

 signals:
  void NormalEstimation(float vpx, float vpy, float vpz);

 public slots:
  void normalsResult(const pca::Cloud::Ptr &cloud, float time);
  void updateNormals();

 private:
  bool checkValid(bool preview = false);

 private:
  Ui::Normals *ui;
  QThread thread;
  pca::Features *feature;
  std::vector<pca::Cloud::Ptr> selected_clouds;
  std::unordered_map<QString, pca::Cloud::Ptr> normals_map;
};

#endif  // NORMALS_H
