#ifndef FACTORY_H
#define FACTORY_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/common/modeltree.h"

namespace Ui {
class Factory;
}

class Factory : public QWidget
{
    Q_OBJECT

public:
    explicit Factory(QWidget *parent = nullptr);
    ~Factory();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;
    ModelTree *modelTree;

    void init();
    void add();
    void apply();
    void reset();

signals:
    void planeCoefficients(const ModelCoefs &coefs);
    void sphereCoefficients(const ModelCoefs &coefs);
    void lineCoefficients(const ModelCoefs &coefs);
    void cyplinderCoefficients(const ModelCoefs &coefs);
    void circleCoefficients(const ModelCoefs &coefs);
    void coneCoefficients(const ModelCoefs &coefs);
    void cubeCoefficients(const ModelCoefs &coefs);
    void arrowCoefficients(const ModelCoefs &coefs);
    void text3DCoefficients(const ModelCoefs &coefs);

public slots:
    void changeModel(const Model &selectedModel);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Factory *ui;
    bool isAdjustable;
    ModelCoefs coefs;
    Model model;
};

#endif // FACTORY_H
