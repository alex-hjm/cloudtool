/**
 * @file console.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */

#include "base/console.h"

#include <QDateTime>

namespace ct
{
    void Console::print(log_level level, const QString &message)
    {
        QString currenttime = QDateTime::currentDateTime().toString("hh:mm:ss");
        QString level_color;
        switch (level)
        {
        case log_level::LOG_INFO:
            level_color = "<font color='black'>" + message + "</font>";
            break;
        case log_level::LOG_WARNING:
            level_color = "<font color='#CFBF17'>" + message + "</font>";
            break;
        case log_level::LOG_ERROR:
            level_color = "<font color='red'>" + message + "</font>";
            break;
        }
        append("[" + currenttime + "]: " + level_color);
        moveCursor(QTextCursor::End);
    }
}