/**
 * @file console.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CONSOLE_H
#define CT_BASE_CONSOLE_H

#include <QTextBrowser>

#include "base/exports.h"

#define LOG_STATU_NO_POINTCLOUD         "please select a pointcloud!"
#define LOG_STATU_ERROR_PARAM           "parameter error!"
#define LOG_STATU_NOT_MATCH             "the selected pointcloud does not match!"

#define LOG_STATU_ADD_DONE(x)           QString(x) + " add done."
#define LOG_STATU_APPLY_DONE(x)         QString(x) + " apply done."
#define LOG_STATU_RESET_DONE(x)         QString(x) + " reset done."
#define LOG_STATU_PREVIEW_DONE(x)       QString(x) + " preview done."  
#define LOG_STATU_PROCESS_DONE(x, y)    QString(x) + " done, take time "+ QString::number(y) + " ms." 

namespace ct
{
    enum log_level
    {
        LOG_INFO,
        LOG_ERROR,
        LOG_WARNING
    };

    class CT_EXPORT Console : public QTextBrowser
    {
        Q_OBJECT
    public:
        explicit Console(QWidget* parent = nullptr) : QTextBrowser(parent) {}

        /**
         * @brief 打印日志
         */
        void print(log_level level, const QString& message);
    };

} // namespace ct
#endif