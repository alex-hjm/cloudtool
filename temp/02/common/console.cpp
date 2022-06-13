#include "console.h"

Console::Console(QWidget *parent) : QTextBrowser(parent),
    dark(false)
{
    this->setStyleSheet("QTextBrowser{font-size: 9pt;font-family:微软雅黑;}");
    this->setFocusPolicy(Qt::NoFocus);
}

void Console::info(QString message)
{

    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString output;
    if(dark)
        output ="<font color='white'>"+message+"</font>";
    else
        output ="<font color='black'>"+message+"</font>";
    QString myhtml="["+currenttime+"]:"+output;
    this->append(myhtml);
    this->moveCursor(QTextCursor::End);
}

void Console::warning(QString message)
{
    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString output="<font color='#CFBF17'>"+message+"</font>";
    QString myhtml="["+currenttime+"]: "+output;
    this->append(myhtml);
    this->moveCursor(QTextCursor::End);
}

void Console::error(QString message)
{
    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString output="<font color='red'>"+message+"</font>";
    QString myhtml="["+currenttime+"]: "+output;
    this->append(myhtml);
    this->moveCursor(QTextCursor::End);
}

void Console::changeTheme(const bool &state)
{
   dark=state;
}


