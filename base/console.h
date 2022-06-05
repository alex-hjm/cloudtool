/**
 * @file console.h
 * @author hjm (hjmalex@163.com)
 * @version 1.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CONSOLE_H
#define CT_BASE_CONSOLE_H

#include <QTextBrowser>

#include "base/exports.h"

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
    explicit Console(QWidget *parent = nullptr) : QTextBrowser(parent) {}

    /**
     * @brief 打印日志
     */
    void print(log_level level, const QString &message);
  };

} // namespace ct
#endif