/**
 * @file shortcutkey.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-09
 */
#ifndef CT_HELP_SHORTCUTKEY_H
#define CT_HELP_SHORTCUTKEY_H

#include <QDialog>

namespace Ui { class ShortcutKey; }

class ShortcutKey : public QDialog
{
  Q_OBJECT

public:
  explicit ShortcutKey(QWidget* parent = nullptr);
  ~ShortcutKey();

private:
  Ui::ShortcutKey* ui;
};

#endif  // CT_HELP_SHORTCUTKEY_H
