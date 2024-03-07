/**
 * @file transforms.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-08
 */
#ifndef _CT_TRANSFORMS_H_
#define _CT_TRANSFORMS_H_

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <QString>

#define MATRIX_SIZE 16
#define EULER_SIZE  6

void getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw) 
{
    pcl::getEulerAngles(t, roll, pitch, yaw);
    roll = pcl::rad2deg(roll);
    pitch = pcl::rad2deg(pitch);
    yaw = pcl::rad2deg(yaw);
}

void getAngleAxis(const Eigen::Affine3f& t, float& angle, float& axisX, float& axisY, float& axisZ)
{
    Eigen::Matrix3f rotation_matrix = t.matrix().topLeftCorner(3, 3);
    Eigen::AngleAxisf angleAxis;
    angleAxis.fromRotationMatrix(rotation_matrix);
    Eigen::Vector3f axis(angleAxis.axis());
    angle = pcl::rad2deg(angleAxis.angle());
    axisX = axis[0];
    axisY = axis[1];
    axisZ = axis[2];
}

void getTranslationAndEulerAngles(const Eigen::Affine3f& t, float& x, float& y, float& z, float& roll, float& pitch, float& yaw)
{
    pcl::getTranslationAndEulerAngles(t, x, y, z, roll, pitch, yaw);
    roll = pcl::rad2deg(roll);
    pitch = pcl::rad2deg(pitch);
    yaw = pcl::rad2deg(yaw);
}

Eigen::Affine3f getTransformation(float x, float y, float z, float roll, float pitch, float yaw)
{
    return pcl::getTransformation(x, y, z, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw));
}

Eigen::Affine3f getTransformation(float angle, float axisX, float axisY, float axisZ, float x, float y, float z)
{
    Eigen::AngleAxisf rotation_vector(pcl::deg2rad(angle), Eigen::Vector3f(axisX, axisY, axisZ));
    Eigen::Vector3f eulerAngle = rotation_vector.matrix().eulerAngles(0, 1, 2);  // r,p,y
    return pcl::getTransformation(x, y, z, eulerAngle[0], eulerAngle[1], eulerAngle[2]);
}

Eigen::Affine3f getTransformation(const QString& text, bool& success)
{
#if (QT_VERSION <= QT_VERSION_CHECK(5,14,0))
    QStringList valuesStr = text.simplified().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
#else
    QStringList valuesStr = text.simplified().split(QRegExp(",|\\s+"), Qt::SplitBehaviorFlags::SkipEmptyParts);
#endif
    Eigen::Affine3f result = Eigen::Affine3f::Identity();
    if (valuesStr.size() == MATRIX_SIZE) {
        Eigen::Matrix4f mat;
        for (int r = 0, idx = 0; r < 4; r++)
            for (int c = 0; c < 4; c++) {
                result.matrix()(r, c) = valuesStr[idx++].toFloat(&success);
                if (!success) return result;
            }

        if (result.matrix()(3, 3) != 1 && result.matrix()(3, 3) != 0)  {
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    result.matrix()(r, c) *= 1.0 / result.matrix()(3, 3);
            result.matrix()(3, 3) = 1;
        }
    }
    else if (valuesStr.size() == EULER_SIZE) {
        bool ok = false;
        float val[6];
        for (int i = 0;i < 6;i++)
        {
            val[i] = valuesStr[0].toFloat(&success);
            if (!success) return result;
        }
        result = getTransformation(val[0], val[1], val[2], val[3], val[4], val[5]);
    }
    success = true;
    return result; 
}

QString getTransformationQString(const Eigen::MatrixXf& mat, int decimals)
{
    QString str;
    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            str += QString::number(mat(i, j), 'f', decimals) + " ";
        }
        if (i < mat.rows() - 1) str += "\n";
    }
    return str;
}

#endif // _CT_TRANSFORMS_H_