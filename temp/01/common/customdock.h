#ifndef CUSTOMDOCK_H
#define CUSTOMDOCK_H

#include <QObject>
#include <QDialog>
#include <QDebug>
#include <QDockWidget>
#include <unordered_map>

#include "panel/colors.h"
#include "panel/coords.h"
#include "panel/about.h"
#include "panel/shortcutkey.h"
#include "panel/normals.h"
#include "panel/scale.h"
#include "panel/boundingbox.h"
#include "panel/transformation.h"
#include "panel/treesearch.h"
#include "panel/filters.h"
#include "panel/calibration.h"
#include "panel/cutting.h"
#include "panel/samplings.h"
#include "panel/measure.h"
#include "panel/surfaces.h"
#include "panel/segmentations.h"
#include "panel/devicekinectdk.h"
#include "panel/devicezhisensor.h"
#include "panel/registrations.h"
#include "panel/descriptor.h"
#include "panel/pointpick.h"

enum left_dock_type
{
    color=0,
    normals,
    boundingbox,
    transformation,
    treesearch,
    filters,
    surfaces,
    descriptor,
    segmentations,
    device_kinectdk,
    device_zhisensor,
    keypoint,
};

enum right_dock_type
{
    calibration=0,
    registrations,
    recognition,

};

enum dialog_type
{
    coords=0,
    scale,
    pointpick,
    cutting,
    samplings,
    measure,
};

class CustomDock : public QObject
{
    Q_OBJECT
public:
    explicit CustomDock(QObject *parent = nullptr):QObject(parent){}
    //left dock
    std::unordered_map<left_dock_type,QDockWidget*> left_dock;
    std::unordered_map<left_dock_type,bool> left_dock_visible;
    //right dock
    std::unordered_map<right_dock_type,QDockWidget*> right_dock;
    std::unordered_map<right_dock_type,bool> right_dock_visible;
    //dialog
    std::unordered_map<dialog_type,QDialog*> dialog;

public:
    template<typename T>
    bool createLeftDock(const left_dock_type &p,QWidget* parent)
    {
        if(left_dock[p]==nullptr){
            left_dock[p]=new T(parent);
            left_dock[p]->setAttribute(Qt::WA_DeleteOnClose);
            connect(left_dock[p], &QDockWidget::destroyed, [=]{
                disconnect(left_dock[p],0,0,0);
                left_dock[p]=nullptr;
            });
            connect(left_dock[p],&QDockWidget::visibilityChanged,[=](bool state){
                left_dock_visible[p]=!state;
            });
            return  true;
        } else {
            if(left_dock_visible[p])
                left_dock[p]->raise();
            else
                left_dock[p]->close();
            return  false;
        }
    }
    template<typename T>
    bool createRightDock(const right_dock_type &p,QWidget* parent)
    {
        if(right_dock[p]==nullptr){
            right_dock[p]=new T(parent);
            right_dock[p]->setAttribute(Qt::WA_DeleteOnClose);
            connect(right_dock[p], &QDockWidget::destroyed, [=]{
                disconnect(right_dock[p],0,0,0);
                right_dock[p]=nullptr;
            });
            connect(right_dock[p],&QDockWidget::visibilityChanged,[=](bool state){
                right_dock_visible[p]=!state;
            });
            return  true;
        } else {
            if(right_dock_visible[p])
                right_dock[p]->raise();
            else
                right_dock[p]->close();
            return  false;
        }
    }

    template<typename T>
    bool createDialog(const dialog_type &p,QWidget* parent)
    {
        if(dialog[p]==nullptr) {
            for(auto &i : dialog)
                if(i.first!=p && i.second != nullptr)
                    return false;
            dialog[p] = new T(parent);
            dialog[p]->setAttribute(Qt::WA_DeleteOnClose);
            dialog[p]->setWindowFlags(Qt::FramelessWindowHint|Qt::Dialog);
            connect(dialog[p], &QDialog::destroyed, [=] {
                disconnect(dialog[p],0,0,0);
                dialog[p]=nullptr;
            });
            dialog[p]->show();
            return  true;
        }else
            dialog[p]->close();
        return false;
    }

    void close()
    {
        for(auto &dock : left_dock)
            if(dock.second != nullptr)
                dock.second->close();
        for(auto &dock : right_dock)
            if(dock.second != nullptr)
                dock.second->close();
        for(auto &i : dialog)
            if(i.second != nullptr)
                i.second->close();
    }
};

#endif // CUSTOMDOCK_H
