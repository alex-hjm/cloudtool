/**
 * @file customdialog.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CUSTOMDIALOG_H
#define CT_BASE_CUSTOMDIALOG_H

#include "base/cloudtree.h"
#include "base/cloudview.h"
#include "base/console.h"
#include "base/exports.h"

#include <QDialog>
#include <QMainWindow>
#include <QPushButton>
#include <QResizeEvent>
#include <map>

namespace ct
{
    class CT_EXPORT CustomDialog : public QDialog
    {
        Q_OBJECT
    public:
        explicit CustomDialog(QWidget* parent = nullptr) : QDialog(parent) {}

        ~CustomDialog() {}

        void setCloudView(CloudView* cloudview) { m_cloudview = cloudview; }

        void setCloudTree(CloudTree* cloudtree) { m_cloudtree = cloudtree; }

        void setConsole(Console* console) { m_console = console; }

        virtual void init() {}

        virtual void reset() {}

        virtual void deinit() {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    signals:
        void sizeChanged(const QSize&);

    protected:

        void printI(const QString& message) { m_console->print(LOG_INFO, message); }
        void printW(const QString& message) { m_console->print(LOG_WARNING, message); }
        void printE(const QString& message) { m_console->print(LOG_ERROR, message); }

        void closeEvent(QCloseEvent* event)
        {
            reset();
            deinit();
            return QDialog::closeEvent(event);
        }

        void resizeEvent(QResizeEvent* event)
        {
            emit sizeChanged(event->size());
            return QDialog::resizeEvent(event);
        }

    public:
        CloudView* m_cloudview;
        CloudTree* m_cloudtree;
        Console* m_console;
    };

    static std::map<QString, CustomDialog*> registed_dialogs;
    static std::map<QString, bool> dialogs_visible;

    /**
     * @brief 创建弹出窗口
     * @param parent 主窗口
     * @param label 窗口标签
     * @param central_pos 中心部件的相对位置
     * @param move_signal 窗口移动信号 void posChanged(const QPoint &pos)
     */
    template <class T>
    void createDialog(QMainWindow* parent, const QString& label, CloudView* cloudview = nullptr,
                      CloudTree* cloudtree = nullptr, Console* console = nullptr)
    {
        if (parent == nullptr) return;
        if (registed_dialogs.find(label) == registed_dialogs.end()) // register dock
            registed_dialogs[label] = nullptr;
        if (registed_dialogs.find(label)->second == nullptr) // create new dialog
        {
            for (auto& dialog : registed_dialogs)
                if (dialog.first != label && dialog.second != nullptr) return;
            registed_dialogs[label] = new T(parent);
            if (cloudview)
                registed_dialogs[label]->setCloudView(cloudview);
            if (cloudtree)
                registed_dialogs[label]->setCloudTree(cloudtree);
            if (console)
                registed_dialogs[label]->setConsole(console);
            registed_dialogs[label]->setAttribute(Qt::WA_DeleteOnClose);
            registed_dialogs[label]->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
            registed_dialogs[label]->init();
            QObject::connect(registed_dialogs[label], &QDialog::destroyed, [=]
                             { registed_dialogs[label] = nullptr; });
            registed_dialogs[label]->show();

            QPoint pos = cloudview->mapToGlobal(QPoint(0, 0));
            registed_dialogs[label]->move(pos.x() + cloudview->width() - registed_dialogs[label]->width() - 9, pos.y() + 9);
            QObject::connect(cloudview, &CloudView::posChanged, [=](const QPoint& pos)
                             {
                                 if (registed_dialogs[label] != nullptr)
                                 {
                                     int ax = pos.x() + cloudview->width() - registed_dialogs[label]->width() - 9;
                                     int ay = pos.y() + 9;
                                     registed_dialogs[label]->move(ax, ay);
                                 }
                             });
            QObject::connect(registed_dialogs[label], &CustomDialog::sizeChanged, [=](const QSize& size)
                             {
                                 if (registed_dialogs[label] != nullptr)
                                 {
                                     QPoint pos = cloudview->mapToGlobal(QPoint(0, 0));
                                     int ax = pos.x() + cloudview->width() - size.width() - 9;
                                     int ay = pos.y() + 9;
                                     registed_dialogs[label]->move(ax, ay);
                                 }
                             });
        }
        else // update dialog
            registed_dialogs[label]->close();
    }

    /**
     * @brief 获取窗口指针
     * @param label 窗口标签
     */
    template <class T>
    T* getDialog(const QString& label)
    {
        if (registed_dialogs.find(label) == registed_dialogs.end())
            return nullptr;
        else
            return (T*)registed_dialogs.find(label)->second;
    }

} // namespace pca

#endif // PCA_CUSTOMDIALOG_H