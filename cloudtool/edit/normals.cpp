/**
 * @file normals.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-05
 */
#include "normals.h"
#include "ui_normals.h"

Normals::Normals(QWidget* parent): QDockWidget(parent), 
    ui(new Ui::Normals)
{
    ui->setupUi(this);
}

Normals::~Normals()
{
    delete ui;
}
