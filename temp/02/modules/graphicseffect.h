﻿#ifndef QCUSTOMGRAPHICSEFFECT_H
#define QCUSTOMGRAPHICSEFFECT_H

#include <QObject>
#include <QGraphicsEffect>
#include <QPainter>

class QCustomGraphicsEffect : public QGraphicsEffect
{
    Q_OBJECT
public:
    explicit QCustomGraphicsEffect(QObject *parent = nullptr);

protected:
    void draw(QPainter *painter);
};

#endif // QCUSTOMGRAPHICSEFFECT_H
