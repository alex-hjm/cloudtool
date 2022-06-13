#ifndef SHORTCUTKEY_H
#define SHORTCUTKEY_H

#include <QDialog>

namespace Ui {
class ShortcutKey;
}

class ShortcutKey : public QDialog
{
    Q_OBJECT

public:
    explicit ShortcutKey(QWidget *parent = nullptr);
    ~ShortcutKey();

private:
    Ui::ShortcutKey *ui;
};

#endif // SHORTCUTKEY_H
