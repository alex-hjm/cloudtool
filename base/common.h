/**
 * @file common.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_COMMON_H
#define CT_BASE_COMMON_H

#include "base/exports.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <QString>

namespace ct
{
	/**
	 * @brief 将颜色从HSV格式转换为RGB格式。
	 */
	void CT_EXPORT HSVtoRGB(float h, float s, float v, float& r, float& g, float& b);

	/**
	 * @brief 从给定的变换中提取欧拉角（内在旋转，ZYX 约定）。
	 */
	void CT_EXPORT getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw);

	/**
	 * @brief 从给定的变换中提取轴角（内在旋转，ZYX 约定）。
	 */
	void CT_EXPORT getAngleAxis(const Eigen::Affine3f& t, float& angle, float& axisX, float& axisY, float& axisZ);

	/**
	 * @brief 从给定的变换中提取 x、y、z 和欧拉角（内在旋转，ZYX 约定）。
	 */
	void CT_EXPORT getTranslationAndEulerAngles(const Eigen::Affine3f& t, float& x, float& y, float& z, float& roll, float& pitch, float& yaw);

	/**
	 * @brief 从给定的平移和欧拉角（内在旋转，ZYX 约定）创建变换
	 */
	void CT_EXPORT getTransformation(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t);

	/**
	 * @brief 从给定的字符串中（内在旋转，ZYX 约定）创建变换
	 * @param text  4x4 matrix  / x y z rx ry rz
	 */
	bool CT_EXPORT getTransformation(const QString& text, Eigen::Affine3f& t);

	/**
	 * @brief 将给定的变换转换成字符串。
	 * @param decimals  有效位数
	 */
	QString CT_EXPORT getTransformationQString(const Eigen::MatrixXf& mat, int decimals);

	/**
	 * @brief 从给定的平移和欧拉角（内在旋转，ZYX 约定）创建变换
	 */
	Eigen::Affine3f CT_EXPORT getTransformation(float x, float y, float z, float roll, float pitch, float yaw);

	/**
	 * @brief 从给定的轴角（内在旋转，ZYX 约定）创建变换
	 */
	Eigen::Affine3f CT_EXPORT getTransformation(float angle, float axisX, float axisY, float axisZ, float x, float y, float z);

	/**
	 * @brief 从给定的欧拉角（内在旋转，ZYX 约定）创建 3x3 旋转矩阵
	 */
	Eigen::Matrix3f CT_EXPORT getRotationMatrix(float roll, float pitch, float yaw);
} // namespace ct
#endif // CT_BASE_COMMON_H