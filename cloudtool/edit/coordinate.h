#ifndef COORDS_H
#define COORDS_H

#include "pca/customdialog.h"

namespace Ui {
class Coordinate;
}

class Coordinate : public pca::CustomDialog {
  Q_OBJECT

 public:
  explicit Coordinate(QWidget *parent = nullptr);
  ~Coordinate();

  virtual void init();
  void add();
  virtual void reset();
  void addCoord();
  void closeCoord();

 private:
  Ui::Coordinate *ui;
  std::vector<pca::Coord> coordinate_vec;
};

#endif  // COORDS_H
