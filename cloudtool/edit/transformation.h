#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include "pca/customdock.h"

namespace Ui {
class Transformation;
}

class Transformation : public pca::CustomDock {
  Q_OBJECT

 public:
  explicit Transformation(QWidget *parent = nullptr);
  ~Transformation();
  virtual void init();
  void add();
  void apply();
  virtual void reset();

 private:
  bool checkValid(bool preview = false);

 signals:
  void affine(const Eigen::Affine3f &);

 public slots:
  void preview(const Eigen::Affine3f &);

 private:
  Ui::Transformation *ui;
  Eigen::Affine3f trans_affine;
  std::vector<pca::Cloud::Ptr> selected_clouds;
  std::unordered_map<QString, Eigen::Affine3f> trans_map;
};

#endif  // TRANSFORMATION_H
