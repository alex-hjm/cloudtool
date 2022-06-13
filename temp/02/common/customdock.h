#ifndef CustomDock_H
#define CustomDock_H

#include <QDockWidget>
#include <QDialog>
#include <unordered_map>

#include "src/panel/transform.h"
#include "src/panel/color.h"
#include "src/panel/factory.h"
#include "src/panel/coordinate.h"
#include "src/panel/normals.h"
#include "src/panel/boundingbox.h"
#include "src/panel/scale.h"
#include "src/panel/treesearch.h"
#include "src/panel/filter.h"
#include "src/panel/segmentation.h"
#include "src/panel/registration.h"
#include "src/panel/recognition.h"
#include "src/panel/surface.h"
#include "src/panel/feature.h"
#include "src/panel/sampling.h"
#include "src/panel/measure.h"
#include "src/panel/segment.h"
#include "src/panel/devicekinectdk.h"
#include "src/panel/devicezhisensor.h"
#include "src/panel/calibration.h"
#include "src/panel/about.h"
#include "src/panel/shortcutkey.h"

class CustomDock: public QWidget
{
    Q_OBJECT
public:
    explicit CustomDock(QWidget *parent = nullptr): QWidget(parent){}
    ~CustomDock(){}
    //left dock
    std::unordered_map<left_panel,QDockWidget*> leftDock;
    std::unordered_map<left_panel,QWidget*> leftWidget;
    std::unordered_map<left_panel,bool> leftPanelisHided;
    //right dock
    std::unordered_map<right_panel,QDockWidget*> rightDock;
    std::unordered_map<right_panel,QWidget*> rightWidget;
    std::unordered_map<right_panel,bool> rightPanelisHided;
    //dialog
    std::unordered_map<dialog_panel,QDialog*> Dialog;

    template<typename T>
    bool createDock(const left_panel &p,const QString &title,QWidget* parent)
    {
        if(leftDock[p]==nullptr) {
            leftWidget[p]=new T();
            leftDock[p]=new QDockWidget(parent);
            leftWidget[p]->setAttribute(Qt::WA_DeleteOnClose);
            leftDock[p]->setAttribute(Qt::WA_DeleteOnClose);
            leftDock[p]->setWidget(leftWidget[p]);
            leftDock[p]->setWindowTitle(title);
            leftPanelisHided[p]=false;
            connect(leftDock[p], &QDockWidget::destroyed, [=]{
                disconnect(leftWidget[p],0,0,0);
                leftWidget[p]->close();
                leftDock[p]=nullptr;
            });
            connect(leftDock[p],&QDockWidget::visibilityChanged,[=](bool state){
                leftPanelisHided[p]=!state;
            });
            return  true;
        } else {
            if(leftPanelisHided[p])
                leftDock[p]->raise();
            else
                leftDock[p]->close();
            return  false;
        }
    }
    template<typename T>
    bool createDock(const right_panel &p,const QString &title,QWidget* parent)
    {
        if(rightDock[p]==nullptr) {
            rightWidget[p] = new T();
            rightDock[p]=new QDockWidget(parent);
            rightWidget[p]->setAttribute(Qt::WA_DeleteOnClose);
            rightDock[p]->setAttribute(Qt::WA_DeleteOnClose);
            rightDock[p]->setWidget(rightWidget[p]);
            rightDock[p]->setWindowTitle(title);
            rightPanelisHided[p]=false;
            connect(rightDock[p], &QDockWidget::destroyed, [=] {
                disconnect(rightWidget[p],0,0,0);
                rightWidget[p]->close();
                rightDock[p]=nullptr;
            });
            connect(rightDock[p],&QDockWidget::visibilityChanged,[=](bool state){
                rightPanelisHided[p]=!state;
            });
            return  true;
        } else {
            if(rightPanelisHided[p])
                rightDock[p]->raise();
            else
                rightDock[p]->close();
            return false;
        }
    }

    template<typename T>
    bool createDialog(const dialog_panel &p,QWidget* parent)
    {
        if(Dialog[p]==nullptr) {
            for(auto panel : Dialog){
                if(panel.first!=p && panel.second != nullptr){
                    return false;
                }
            }
            Dialog[p] = new T(parent);
            Dialog[p]->setAttribute(Qt::WA_DeleteOnClose);
            Dialog[p]->setWindowFlags(Qt::FramelessWindowHint|Qt::Dialog);
            connect(Dialog[p], &QDialog::destroyed, [=] {
                disconnect(Dialog[p],0,0,0);
                disconnect(this,0,0,0);
                Dialog[p]=nullptr;
            });
            connect(this,&CustomDock::posChanged,[=](int x,int y){
               Dialog[p]->move(x-Dialog[p]->width(),y);
            });
            Dialog[p]->show();
            return  true;
        }else
            Dialog[p]->close();
        return false;
    }

    void closeAllDock()
    {
        for(auto panel : rightDock){
            if(panel.second != nullptr){
                panel.second->close();
            }
        }
        for(auto panel : leftDock){
            if(panel.second != nullptr){
                panel.second->close();
            }
        }
    }

signals:
    void posChanged(int x,int y);
};

#endif // CustomDock_H
