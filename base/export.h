/**
 * @file export.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-28
 */
#ifndef __BASE_EXPORT_H__
#define __BASE_EXPORT_H__

#include <QtCore/qglobal.h>

#if defined(CT_LIBRARY)
#define CT_EXPORT Q_DECL_EXPORT
#else
#define CT_EXPORT Q_DECL_IMPORT
#endif

#define CT_NAMESPACE        ct
#define CT_BEGIN_NAMESPACE  namespace CT_NAMESPACE  {
#define CT_END_NAMESPACE    }

#endif // __BASE_EXPORT_H__