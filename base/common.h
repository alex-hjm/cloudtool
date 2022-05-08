/**
 * @file common.h
 * @author hjm (hjmalex@163.com)
 * @version 1.0
 * @date 2022-05-08
 *
 * @copyright Copyright (c) 2022 hjm
 *
 */
#ifndef CT_BASE_COMMON_H
#define CT_BASE_COMMON_H

#include "base/exports.h"

namespace ct {
/**
 * @brief 将颜色从HSV格式转换为RGB格式。
 */
void CT_EXPORT HSVtoRGB(float h, float s, float v, float &r, float &g,
                        float &b);
}  // namespace ct
#endif  // CT_BASE_COMMON_H