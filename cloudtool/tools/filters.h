#ifndef FILTERS_H
#define FILTERS_H

#include <QThread>

#include "pca/customdock.h"
#include "pca/filters.h"

namespace Ui {
class Filters;
}

class Filters : public pca::CustomDock {
  Q_OBJECT

 public:
  explicit Filters(QWidget *parent = nullptr);
  ~Filters();
  virtual void init();
  void preview();
  void add();
  void apply();
  virtual void reset();

 signals:
  void ConditionalRemoval(pca::Condition::Ptr con);

 public slots:
  void filterResult(const pca::Cloud::Ptr &cloud, float time);

 private:
  bool checkValid(bool preview = false);
  pca::Condition::Ptr getCondition();

 private:
  Ui::Filters *ui;
  QThread thread;
  pca::Filters *filters;
  std::vector<pca::Cloud::Ptr> selected_clouds;
  std::unordered_map<QString, pca::Cloud::Ptr> filtered_map;
};

#endif  // FILTERS_H
