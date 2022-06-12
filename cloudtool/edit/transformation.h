/**
 * @file transformation.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_EDIT_TRANSFORMATION_H
#define CT_EDIT_TRANSFORMATION_H

#include "base/customdock.h"

namespace Ui
{
    class Transformation;
}

class Transformation : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit Transformation(QWidget* parent = nullptr);
    ~Transformation();

    void add();
    void apply();
    virtual void reset();

signals:
    void affine(const Eigen::Affine3f& affine);

public slots:
    void preview(const Eigen::Affine3f& affine);

private:
    Ui::Transformation* ui;
    Eigen::Affine3f m_affine;
    std::unordered_map<QString, Eigen::Affine3f> m_trans_map;
};

#endif  // CT_EDIT_TRANSFORMATION_H
