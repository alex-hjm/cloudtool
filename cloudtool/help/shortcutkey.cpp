#include "shortcutkey.h"

#include "ui_shortcutkey.h"

ShortcutKey::ShortcutKey(QWidget* parent) : QDialog(parent), ui(new Ui::ShortcutKey)
{
  ui->setupUi(this);
}

ShortcutKey::~ShortcutKey() { delete ui; }
