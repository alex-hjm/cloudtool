/**
 * @file exports.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_EXPORTS_H
#define CT_BASE_EXPORTS_H

#include <QtCore/qglobal.h>

#if defined(CT_LIBRARY)
#  define CT_EXPORT Q_DECL_EXPORT
#else
#  define CT_EXPORT Q_DECL_IMPORT
#endif

#endif // CT_BASE_EXPORTS_H