/**
 * @file console.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-24
 */
#include "console.h"

#include <QDateTime>

CT_BEGIN_NAMESPACE

Console::Console(QWidget* parent) : QTextBrowser(parent)
{
    
}

void Console::logging(LogLevel level, const QString& msg)
{
    QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString level_color;
    switch (level) {
        case LogLevel::LOG_INFO:
            level_color = "[INFO]: <font color='black'>" + msg + "</font>";
            break;
        case LogLevel::LOG_WARN:
            level_color = "[WARN]: <font color='#FFC90E'>" + msg + "</font>";
            break;
        case LogLevel::LOG_ERROR:
            level_color = "[ERROR]: <font color='red'>" + msg + "</font>";
            break;
    }
    append("[" + currenttime + "]" + level_color);
    moveCursor(QTextCursor::End);
}

CT_END_NAMESPACE