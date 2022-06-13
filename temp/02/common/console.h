#ifndef CONSOLE_H
#define CONSOLE_H

#include <QWidget>
#include <QDateTime>
#include <QTextBrowser>
#include <QHBoxLayout>

class Console : public QTextBrowser
{
    Q_OBJECT
public:
    explicit Console(QWidget *parent = nullptr);

    void info(QString message);
    void warning(QString message);
    void error(QString message);
    void changeTheme(const bool &state);

private:
    bool dark;
    QSize sizeHint() const
    {   return QSize(1236, 90);}
};

#endif // CONSOLE_H
