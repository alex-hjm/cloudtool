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

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QWidget widget;
    ct::CloudList cl(&widget);
    cl.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    ct::CloudView cv(&widget);
    cv.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    QTextBrowser tb(&widget);
    tb.setSizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Ignored);
    QObject::connect(&cl, &ct::CloudList::logging, [&](ct::LogLevel level, const QString& msg) {
       tb.append(msg);
    });
    QObject::connect(&cl, &ct::CloudList::addCloudEvent, &cv, &ct::CloudView::addCloud);
    QObject::connect(&cl, &ct::CloudList::removeCloudEvent, &cv, &ct::CloudView::removeCloud);
    QObject::connect(&cl, &ct::CloudList::addCloudBBoxEvent, &cv, &ct::CloudView::addCloudBBox);
    QObject::connect(&cl, &ct::CloudList::removeCloudBBoxEvent, &cv, &ct::CloudView::removeCloudBBox);
    QVBoxLayout vlayout(&widget);
    vlayout.addWidget(&cv);
    vlayout.addWidget(&cl);
    vlayout.addWidget(&tb);
    widget.setLayout(&vlayout);
    widget.setMinimumSize(200, 200);
    widget.show();
    return a.exec();
}