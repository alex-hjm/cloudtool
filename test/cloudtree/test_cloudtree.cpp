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

#include "base/cloudtree.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QWidget widget;
    ct::CloudTree ct(&widget);
    QTextBrowser tb(&widget);
    QObject::connect(&ct, &ct::CloudTree::logging, [&](ct::LogLevel level, const QString& msg) {
       tb.append(msg);
    });
    QVBoxLayout layout(&widget);
    layout.addWidget(&ct);
    layout.addWidget(&tb);
    widget.setLayout(&layout);
    widget.show();
    return a.exec();
}