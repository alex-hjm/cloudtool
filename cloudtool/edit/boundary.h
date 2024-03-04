/**
 * @file boundary.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-05
 */
#ifndef _CT_BOUNDARY_H_
#define _CT_BOUNDARY_H_

#include <QDialog>

namespace Ui { class Boundary; }

class Boundary : public QDialog
{
    Q_OBJECT
public:
    explicit Boundary(QWidget* parent = nullptr);
    ~Boundary();

private:
    Ui::Boundary* ui;
};

#endif  //_CT_BOUNDARY_H_
