/**
 * @file coordinate.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-07
 */
#ifndef _CT_COORDINATE_H_
#define _CT_COORDINATE_H_

#include "common/customwidget.h"

namespace Ui { class Coordinate; }

class Coordinate : public CustomDock
{
    Q_OBJECT
public:
    explicit Coordinate(QWidget* parent = nullptr);
    ~Coordinate();
    
    void add();
    void remove();
    void clear();

    void handleLanguageChanged() override;
    
private:
    Ui::Coordinate* ui;
};

#endif  // _CT_COORDINATE_H_
