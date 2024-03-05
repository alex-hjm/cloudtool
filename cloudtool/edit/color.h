/**
 * @file color.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-06
 */
#ifndef _CT_COLOR_H_
#define _CT_COLOR_H_

#include <common/customwidget.h>

namespace Ui { class Color; }

class Color: public CustomDock
{
    Q_OBJECT
public:
    explicit Color(QWidget* parent = nullptr);
    ~Color();

    void apply();
    void reset();

    void handleLanguageChanged() override;

private:
    void setColor(const QColor& rgb);

private:
    Ui::Color* ui;
    QColor m_rgb;
};

#endif // _CT_COLOR_H_
