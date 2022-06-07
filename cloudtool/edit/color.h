#ifndef COLORS_H
#define COLORS_H

#include <QColorDialog>

#include "pca/customdock.h"

const QColor colors[5][10] = {
    {QColor("#ffffff"), QColor("#e5e5e7"), QColor("#cccccc"), QColor("#9a9a9a"),
     QColor("#7f7f7f"), QColor("#666666"), QColor("#4c4c4c"), QColor("#333333"),
     QColor("#191919"), QColor("#000000")},

    {QColor("#ffd6e7"), QColor("#ffccc7"), QColor("#ffe7ba"), QColor("#ffffb8"),
     QColor("#f4ffb8"), QColor("#d9f7be"), QColor("#b5f5ec"), QColor("#bae7ff"),
     QColor("#d6e4ff"), QColor("#efdbff")},

    {QColor("#ff85c0"), QColor("#ff7875"), QColor("#ffc069"), QColor("#fff566"),
     QColor("#d3f261"), QColor("#95de64"), QColor("#5cdbd3"), QColor("#69c0ff"),
     QColor("#85a5ff"), QColor("#b37feb")},

    {QColor("#f759ab"), QColor("#ff4d4f"), QColor("#ffa940"), QColor("#ffdf3d"),
     QColor("#a0d911"), QColor("#52c41a"), QColor("#13c2c2"), QColor("#1890ff"),
     QColor("#2f54eb"), QColor("#722ed1")},

    {QColor("#9e1068"), QColor("#a8171b"), QColor("#ad4e00"), QColor("#ad8b00"),
     QColor("#5b8c00"), QColor("#006075"), QColor("#006d75"), QColor("#0050b3"),
     QColor("#10239e"), QColor("#391085")}};

namespace Ui {
class Color;
}

class Color : public pca::CustomDock {
  Q_OBJECT

 public:
  explicit Color(QWidget *parent = nullptr);
  ~Color();

  virtual void init();
  void apply();
  virtual void reset();

 signals:
  void rgb(int r, int g, int b);
  void fieldName(const QString &field);

 private:
  bool checkValid();

 private:
  Ui::Color *ui;
  QString field_name;
  int red, green, blue;
  std::vector<pca::Cloud::Ptr> selected_clouds;
};

#endif  // COLORS_H
