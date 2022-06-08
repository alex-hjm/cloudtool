/**
 * @file about.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-09
 */
#include "about.h"

#include "ui_about.h"

About::About(QWidget *parent) : QDialog(parent), ui(new Ui::About)
{
    ui->setupUi(this);
}

About::~About() { delete ui; }
