/**
 * @file customdock.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CUSTOMDOCK_H
#define CT_BASE_CUSTOMDOCK_H

#include "base/cloudtree.h"
#include "base/cloudview.h"
#include "base/console.h"
#include "base/exports.h"

#include <QDockWidget>
#include <QMainWindow>
#include <QPushButton>
#include <map>
#include <set>

namespace ct
{
    class CT_EXPORT CustomDock : public QDockWidget
    {
        Q_OBJECT
    public:
        explicit CustomDock(QWidget* parent = nullptr) : QDockWidget(parent) {}

        ~CustomDock() {}

        void setCloudView(CloudView* cloudview) { m_cloudview = cloudview; }

        void setCloudTree(CloudTree* cloudtree) { m_cloudtree = cloudtree; }

        void setConsole(Console* console) { m_console = console; }

        virtual void init() {}

        virtual void reset() {}

        virtual void deinit() {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        void printI(const QString& message) { m_console->print(LOG_INFO, message); }
        void printW(const QString& message) { m_console->print(LOG_WARNING, message); }
        void printE(const QString& message) { m_console->print(LOG_ERROR, message); }

        void closeEvent(QCloseEvent* event)
        {
            reset();
            deinit();
            return QDockWidget::closeEvent(event);
        }

    public:
        CloudView* m_cloudview;
        CloudTree* m_cloudtree;
        Console* m_console;
    };

    static std::map<QString, CustomDock*> registed_docks;
    static std::set<QString> left_label;
    static std::set<QString> right_label;
    static std::map<QString, bool> docks_visible;

    /**
     * @brief 创建停靠窗口
     * @param parent 主窗口
     * @param label 窗口标签
     * @param area 停靠区域  left/right/top/bottom
     * @param dock 合并的停靠窗口
     */
    template <class T>
    void createDock(QMainWindow* parent, const QString& label, CloudView* cloudview = nullptr,
                    CloudTree* cloudtree = nullptr, Console* console = nullptr,
                    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea, QDockWidget* dock = nullptr)
    {
        if (parent == nullptr) return;
        if (registed_docks.find(label) == registed_docks.end()) // register dock
            registed_docks[label] = nullptr;
        if (registed_docks.find(label)->second == nullptr) // create new dock
        {
            registed_docks[label] = new T(parent);
            if (cloudview)
                registed_docks[label]->setCloudView(cloudview);
            if (cloudtree)
                registed_docks[label]->setCloudTree(cloudtree);
            if (console)
                registed_docks[label]->setConsole(console);
            registed_docks[label]->setAttribute(Qt::WA_DeleteOnClose);
            registed_docks[label]->init();
            QObject::connect(registed_docks[label], &QDockWidget::visibilityChanged, [=](bool state)
                             { docks_visible[label] = !state; });
            QObject::connect(registed_docks[label], &QDockWidget::destroyed, [=]
                             { registed_docks[label] = nullptr; });
            parent->addDockWidget(area, registed_docks[label]);
            if (area == Qt::LeftDockWidgetArea)
                left_label.insert(label);
            else
                right_label.insert(label);
            if (dock == nullptr)
                for (auto& dock : registed_docks)
                {
                    //left
                    if (dock.first != label && (left_label.count(dock.first) > 0) &&
                        area == Qt::LeftDockWidgetArea && dock.second != nullptr)
                    {
                        parent->tabifyDockWidget(dock.second, registed_docks[label]);
                        break;
                    }
                    if (dock.first != label && (right_label.count(dock.first) > 0) &&
                        area == Qt::RightDockWidgetArea && dock.second != nullptr)
                    {
                        parent->tabifyDockWidget(dock.second, registed_docks[label]);
                        break;
                    }
                }
            else
                parent->tabifyDockWidget(dock, registed_docks[label]);
            registed_docks[label]->setVisible(true);
            registed_docks[label]->raise();
        }
        else // update dock
        {
            if (docks_visible.find(label) == docks_visible.end()) return;
            if (docks_visible.find(label)->second)
                registed_docks[label]->raise();
            else
            {
                registed_docks[label]->close();
                registed_docks.erase(label);
            }
        }
    }

    /**
     * @brief 获取窗口指针
     * @param label 窗口标签
     */
    template <class T>
    T* getDock(const QString& label)
    {
        if (registed_docks.find(label) == registed_docks.end())
            return nullptr;
        else
            return (T*)registed_docks.find(label)->second;
    }

} // namespace ct

#endif // CT_BASE_CUSTOMDOCK_H
