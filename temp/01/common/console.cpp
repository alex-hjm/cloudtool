#include "console.h"

Console::Console(QWidget *parent) : QTextBrowser(parent),
    progress_bar(nullptr)
{
    setStyleSheet("QTextBrowser{font-size: 9pt;font-family:微软雅黑;}");
    setFocusPolicy(Qt::NoFocus);
}

Console::~Console()
{
    if(progress_bar!=nullptr) {
        progress_bar->close();
        progress_bar=nullptr;
    }
}

void Console::info(const QString &message)
{

    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString output="<font color='black'>"+message+"</font>";
    QString myhtml="["+currenttime+"]: "+output;
    append(myhtml);
    moveCursor(QTextCursor::End);
}

void Console::warning(const QString &message)
{
    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString output="<font color='#CFBF17'>"+message+"</font>";
    QString myhtml="["+currenttime+"]: "+output;
    append(myhtml);
    moveCursor(QTextCursor::End);
}

void Console::error(const QString &message)
{
    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString output="<font color='red'>"+message+"</font>";
    QString myhtml="["+currenttime+"]: "+output;
    append(myhtml);
    moveCursor(QTextCursor::End);
}

void Console::cout(const QString &message)
{
    append(message);
    moveCursor(QTextCursor::End);
}

void Console::cout(const QString &color, const QString &message)
{
    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString output="<font color='"+color+"'>"+message+"</font>";
    QString myhtml="["+currenttime+"]: "+output;
    append(myhtml);
    moveCursor(QTextCursor::End);
}

void Console::showStatusMessage(const QString &text, int timeout)
{
    status_bar->showMessage(text,timeout);
}

void Console::clearStatusMessage()
{
    status_bar->clearMessage();
}

void Console::showProgressBar()
{
    if(progress_bar==nullptr) {
        progress_bar=new QProgressBar(parent);
        progress_bar->setTextVisible(false);
        progress_bar->setFixedSize(260,20);
        progress_bar->setAttribute(Qt::WA_DeleteOnClose);
        progress_bar->setAttribute(Qt::WA_TranslucentBackground);
        progress_bar->setWindowFlags(Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        progress_bar->setRange(0,0);
        progress_bar->setOrientation(Qt::Horizontal);
    }
    progress_bar->show();
    progress_bar->move((parent->width()-progress_bar->width())/2,(parent->height()-progress_bar->height())/2);
}

void Console::closeProgressBar()
{
    if(progress_bar==nullptr)return;
    progress_bar->close();
    progress_bar=nullptr;
}

void Console::setProgressBarParent(QWidget *par)
{
    parent=par;
}




