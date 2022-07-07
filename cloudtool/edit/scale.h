/**
 * @file scale.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_EDIT_SCALE_H
#define CT_EDIT_SCALE_H

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
    std::map<QString, ct::Cloud::Ptr> m_scale_map;
};

#endif  // CT_EDIT_SCALE_H
