/**
 * @file test_cloudtree.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-30
 */
#include <QApplication>
#include <QVBoxLayout>
#include <iostream>

#include "base/cloudlist.h"
#include "base/cloudview.h"
#include "base/cloudtable.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QWidget widget;
    ct::CloudList cl(&widget);
    cl.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    ct::CloudView cv(&widget);
    cv.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    ct::CloudTable ct(&widget);
    ct.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);

    QObject::connect(&cl, &ct::CloudList::addCloudEvent, &cv, &ct::CloudView::addCloud);
    QObject::connect(&cl, &ct::CloudList::removeCloudEvent, &cv, &ct::CloudView::removeCloud);
    QObject::connect(&cl, &ct::CloudList::addCloudBBoxEvent, &cv, &ct::CloudView::addCloudBBox);
    QObject::connect(&cl, &ct::CloudList::removeCloudBBoxEvent, &cv, &ct::CloudView::removeCloudBBox);
    QObject::connect(&cl, &ct::CloudList::selectCloudEvent, &ct, &ct::CloudTable::handleSelectCloud);
    QObject::connect(&ct, &ct::CloudTable::updateCloudEvent, &cv, &ct::CloudView::addCloud);

    QVBoxLayout vlayout(&widget);
    vlayout.addWidget(&cv);
    vlayout.addWidget(&cl);
    vlayout.addWidget(&ct);
    widget.setLayout(&vlayout);
    widget.setMinimumSize(200, 400);
    widget.show();
    return a.exec();
}