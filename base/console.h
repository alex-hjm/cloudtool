/**
 * @file console.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-24
 */
#ifndef __BASE_CONSOLE_H__
#define __BASE_CONSOLE_H__

#include "types.h"
#include <QTextBrowser>

CT_BEGIN_NAMESPACE

enum LogLevel {LOG_INFO, LOG_WARN, LOG_ERROR};

class CT_EXPORT Console : public QTextBrowser
{
    Q_OBJECT
public:

    explicit Console(QWidget* parent = nullptr);

public slots:
    void logging(LogLevel level, const QString& msg);
};

CT_END_NAMESPACE
#endif