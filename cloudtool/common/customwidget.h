/**
 * @file customwidget.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-06
 */
#ifndef _CT_CUSTOMWIDGET_H_
#define _CT_CUSTOMWIDGET_H_

#include "base/cloudlist.h"
#include "base/cloudview.h"
#include "base/console.h"

#include <QDockWidget>
#include <QDialog>

class CustomDock : public QDockWidget
{
    Q_OBJECT
public:
    explicit CustomDock(QWidget* parent = nullptr) : QDockWidget(parent) {}
    ~CustomDock() {}

    void setCloudView(ct::CloudView* cloudview) { m_cloudview = cloudview; }
    void setCloudList(ct::CloudList* cloudlist) { m_cloudlist = cloudlist; }
    void setConsole(ct::Console* console) { m_console = console; }
    
    virtual void handleLanguageChanged() = 0;

public:
    ct::CloudView* m_cloudview;
    ct::CloudList* m_cloudlist;
    ct::Console* m_console;
};

class CustomDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CustomDialog(QWidget* parent = nullptr) : QDialog(parent) {}
    ~CustomDialog() {}

    void setCloudView(ct::CloudView* cloudview) { m_cloudview = cloudview; }
    void setCloudList(ct::CloudList* cloudlist) { m_cloudlist = cloudlist; }
    void setConsole(ct::Console* console) { m_console = console; }
    
    virtual void handleLanguageChanged() = 0;

public:
    ct::CloudView* m_cloudview;
    ct::CloudList* m_cloudlist;
    ct::Console* m_console;
};


#endif // _CT_CUSTOMWIDGET_H_
