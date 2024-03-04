/**
 * @file transformation.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-05
 */
#ifndef _CT_TRANSFORMATION_H_
#define _CT_TRANSFORMATION_H_

#include <QDockWidget>

namespace Ui { class Transformation;}

class Transformation : public QDockWidget
{
    Q_OBJECT
public:
    explicit Transformation(QWidget* parent = nullptr);
    ~Transformation();

private:
    Ui::Transformation* ui;
};


#endif  // _CT_TRANSFORMATION_H_
