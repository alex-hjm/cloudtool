#ifndef CONSOLE_H
#define CONSOLE_H

#include <QWidget>
#include <QDebug>
#include <QDateTime>
#include <QApplication>
#include <QDesktopWidget>
#include <QStatusBar>
#include <QProgressBar>
#include <QTextBrowser>

class Console : public QTextBrowser
{
    Q_OBJECT
public:
    explicit Console(QWidget *parent = nullptr);
    ~Console();
    void info(const QString &message);
    void warning(const QString &message);
    void error(const QString &message);
    void cout(const QString &message);
    void cout(const QString &color, const QString &message);
    void showStatusMessage(const QString &text, int timeout);
    void clearStatusMessage();
    void showProgressBar();
    void closeProgressBar();
    void setProgressBarParent(QWidget *par);

public:
     QStatusBar *status_bar;
private:
     QProgressBar *progress_bar;
     QWidget *parent;
};

#endif // CONSOLE_H
