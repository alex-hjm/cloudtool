/**
 * @file color.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-12
 */
#ifndef CT_EDIT_COLORS_H
#define CT_EDIT_COLORS_H

#include <QColorDialog>

#include "base/customdock.h"

namespace Ui
{
    class Color;
}

class Color : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit Color(QWidget* parent = nullptr);
    ~Color();

    void apply();
    virtual void reset();

signals:
    void rgb(const QColor& rgb);
    void field(const QString& field);

public slots:
    void setColor(const QColor& rgb);
    void setColor(const QString& field);

private:
    Ui::Color* ui;
    QString m_field;
    QColor m_rgb;
};

#endif // CT_EDIT_COLORS_H
