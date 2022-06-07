/**
 * @file shortcutkey.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-09
 */
#include "shortcutkey.h"

#include "ui_shortcutkey.h"

ShortcutKey::ShortcutKey(QWidget* parent) : QDialog(parent), ui(new Ui::ShortcutKey)
{
  ui->setupUi(this);
}

ShortcutKey::~ShortcutKey() { delete ui; }
