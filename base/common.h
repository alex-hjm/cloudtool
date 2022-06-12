/**
 * @file common.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_COMMON_H
#define CT_BASE_COMMON_H

#include "base/exports.h"

#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>

#include <QString>

namespace ct
{
	void CT_EXPORT HSVtoRGB(float h, float s, float v, float &r, float &g, float &b);
	bool CT_EXPORT getTransformation(const QString& matrix, Eigen::Affine3f &affine);
} // namespace ct
#endif // CT_BASE_COMMON_H