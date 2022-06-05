/**
 * @file customdock.h
 * @author hjm (hjmalex@163.com)
 * @version 1.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CUSTOMDIALOG_H
#define CT_BASE_CUSTOMDIALOG_H

#include <QDialog>
#include <QMainWindow>
#include <QPushButton>
#include <unordered_map>

#include "base/cloudtree.h"
#include "base/cloudview.h"
#include "base/console.h"

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

  protected:
    void closeEvent(QCloseEvent* event)
    {
      reset();
      return QDialog::closeEvent(event);
    }

  public:
    CloudView* m_cloudview;
    CloudTree* m_cloudtree;
    Console* m_console;
  };

  static std::unordered_map<QString, CustomDialog*> registed_dialogs;
  static std::unordered_map<QString, bool> dialogs_visible;

  /**
   * @brief 创建弹出窗口
   * @param parent 主窗口
   * @param label 窗口标签
   * @param central_pos 中心部件的相对位置
   * @param move_signal 窗口移动信号 void posChanged(const QPoint &pos)
   */
  template <typename T, typename Func>
  void createDialog(typename QtPrivate::FunctionPointer<Func>::Object* parent,
                    Func signal, const QString& label, const QPoint& central_pos,
                    CloudView* cloudview = nullptr, CloudTree* cloudtree = nullptr,
                    Console* console = nullptr) {
    if (parent == nullptr) return;
    if (registed_dialogs.find(label) == registed_dialogs.end())  // register dock
      registed_dialogs[label] = nullptr;
    if (registed_dialogs.find(label)->second == nullptr)  // create new dialog
    {
      for (auto& dialog : registed_dialogs)
        if (dialog.first != label && dialog.second != nullptr) return;
      registed_dialogs[label] = new T(parent);
      if (cloudview) registed_dialogs[label]->setCloudView(cloudview);
      if (cloudtree) registed_dialogs[label]->setCloudTree(cloudtree);
      if (console) registed_dialogs[label]->setConsole(console);
      registed_dialogs[label]->setAttribute(Qt::WA_DeleteOnClose);
      registed_dialogs[label]->setWindowFlags(Qt::FramelessWindowHint |
                                              Qt::Dialog);
      registed_dialogs[label]->init();
      QObject::connect(registed_dialogs[label], &QDialog::destroyed, [=]
                       { registed_dialogs[label] = nullptr; });
      registed_dialogs[label]->show();

      QPoint pos = parent->mapToParent(central_pos);
      registed_dialogs[label]->move(pos.x() + cloudview->width() - registed_dialogs[label]->width(), pos.y() + 18);
      QObject::connect(parent, signal, [=](const QPoint& pos)
                       {
                         if (registed_dialogs[label] != nullptr)
                           registed_dialogs[label]->move(pos.x() + cloudview->width() -
                                                         registed_dialogs[label]->width(), pos.y() + 18);
                       });
    }
    else  // update dialog
      registed_dialogs[label]->close();
  }

}  // namespace pca

#endif  // PCA_CUSTOMDIALOG_H