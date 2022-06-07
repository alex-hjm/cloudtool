#ifndef SCALE_H
#define SCALE_H

#include "pca/customdialog.h"

namespace Ui {
class Scale;
}

class Scale : public pca::CustomDialog {
  Q_OBJECT

 public:
  explicit Scale(QWidget *parent = nullptr);
  ~Scale();

  virtual void init();
  void add();
  void apply();
  virtual void reset();

 signals:
  void scale(double x, double y, double z);

 public slots:
  void preview(double x, double y, double z);

 private:
  bool checkValid(bool preview = false);

 private:
  Ui::Scale *ui;
  std::vector<pca::Cloud::Ptr> selected_clouds;
  std::unordered_map<QString, pca::Cloud::Ptr> scaled_clouds_map;
};

#endif  // SCALE_H
