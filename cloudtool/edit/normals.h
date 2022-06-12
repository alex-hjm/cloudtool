/**
 * @file normals.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_EDIT_NORMALS_H
#define CT_EDIT_NORMALS_H

#include <QThread>

#include "base/customdock.h"
#include "modules/features.h"

namespace Ui
{
  class Normals;
}

class Normals : public ct::CustomDock
{
  Q_OBJECT

public:
  explicit Normals(QWidget* parent = nullptr);
  ~Normals();

  void preview();
  void add();
  void apply();
  virtual void reset();

signals:
  void normalEstimation(float vpx, float vpy, float vpz);

public slots:
  void reverseNormals();
  void updateNormals();
  void normalsResult(const ct::Cloud::Ptr& cloud, float time);
  
private:
  Ui::Normals* ui;
  QThread m_thread;
  ct::Features* m_feature;
  std::unordered_map<QString, ct::Cloud::Ptr> m_normals_map;
};

#endif  // NORMALS_H
