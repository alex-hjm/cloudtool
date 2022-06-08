/**
 * @file boundingbox.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-12
 */
#ifndef CT_EDIT_BOUNDINGBOX_H
#define CT_EDIT_BOUNDINGBOX_H

#include "base/customdock.h"

namespace Ui
{
    class BoundingBox;
}

class BoundingBox : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit BoundingBox(QWidget *parent = nullptr);
    ~BoundingBox();

    void preview();
    void apply();
    virtual void reset();

signals:
    void eulerAngles(float r, float p, float y);

public slots:
    void adjustEnable(bool state);
    void adjustBox(float r, float p, float y);

private:
    Ui::BoundingBox *ui;
    int m_box_type;
    std::unordered_map<QString, ct::Box> m_boxs_map;
};

#endif // CT_EDIT_BOUNDINGBOX_H
