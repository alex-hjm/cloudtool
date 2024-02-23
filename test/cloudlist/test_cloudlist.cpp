/**
 * @file test_cloudtree.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-30
 */
#include <QApplication>
#include <QTextBrowser>
#include <QVBoxLayout>
#include <iostream>

#include "base/cloudlist.h"
#include "base/cloudview.h"
#include "base/console.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QWidget widget;
    ct::CloudList cl(&widget);
    cl.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    ct::CloudView cv(&widget);
    cv.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    ct::Console co(&widget);
    co.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    
    QObject::connect(&cl, &ct::CloudList::logging, &co, &ct::Console::logging);
    QObject::connect(&cl, &ct::CloudList::addCloudEvent, &cv, &ct::CloudView::addCloud);
    QObject::connect(&cl, &ct::CloudList::removeCloudEvent, &cv, &ct::CloudView::removeCloud);
    QObject::connect(&cl, &ct::CloudList::addCloudBBoxEvent, &cv, &ct::CloudView::addCloudBBox);
    QObject::connect(&cl, &ct::CloudList::removeCloudBBoxEvent, &cv, &ct::CloudView::removeCloudBBox);

    QVBoxLayout vlayout(&widget);
    vlayout.addWidget(&cv);
    vlayout.addWidget(&cl);
    vlayout.addWidget(&co);
    widget.setLayout(&vlayout);
    widget.setMinimumSize(200, 400);
    widget.show();
    return a.exec();
}