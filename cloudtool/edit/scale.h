/**
 * @file scale.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef SCALE_H
#define SCALE_H

#include "base/customdialog.h"

namespace Ui
{
    class Scale;
}

class Scale : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Scale(QWidget* parent = nullptr);
    ~Scale();

    void add();
    void apply();
    virtual void reset();

signals:
    void scale(double x, double y, double z);

public slots:
    void preview(double x, double y, double z);

private:
    Ui::Scale* ui;
    std::unordered_map<QString, ct::Cloud::Ptr> m_scale_map;
};

#endif  // SCALE_H
