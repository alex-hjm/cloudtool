/**
 * @file normals.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-05
 */
#ifndef _CT_NORMALS_H_
#define _CT_NORMALS_H_

#include <QDockWidget>

namespace Ui { class Normals;}

class Normals : public QDockWidget
{
  Q_OBJECT
public:
  explicit Normals(QWidget* parent = nullptr);
  ~Normals();

private:
  Ui::Normals* ui;
};

#endif  // _CT_NORMALS_H_
