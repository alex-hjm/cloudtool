/**
 * @file transformation.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-05
 */
#include "transformation.h"
#include "ui_transformation.h"


Transformation::Transformation(QWidget* parent): QDockWidget(parent), 
    ui(new Ui::Transformation)
{
    ui->setupUi(this);
}

Transformation::~Transformation() 
{ 
    delete ui; 
}

