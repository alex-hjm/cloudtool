/**
 * @file boundary.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-05
 */
#include "boundary.h"
#include "ui_boundary.h"
#include <QStyleOption>
#include <QPainter>

Boundary::Boundary(QWidget* parent) :QDialog(parent), 
    ui(new Ui::Boundary)
{
    ui->setupUi(this);
    connect(ui->btn_close, &QPushButton::clicked, this, &Boundary::close);
}

Boundary::~Boundary()
{
    delete ui;
}


