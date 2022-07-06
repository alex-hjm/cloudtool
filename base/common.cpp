/**
 * @file common.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/common.h"

#include <QStringList>

#define MATRIX_SIZE 16
#define EULER_SIZE  6

namespace ct
{
    void HSVtoRGB(float h, float s, float v, float& r, float& g, float& b)
    {
        if (s == 0.0f)
        {
            r = g = b = v;
            return;
        }

        h = fmodf(h, 1.0f) / (60.0f / 360.0f);
        int i = (int)h;
        float f = h - (float)i;
        float p = v * (1.0f - s);
        float q = v * (1.0f - s * f);
        float t = v * (1.0f - s * (1.0f - f));

        switch (i)
        {
        case 0:
            r = v, g = t, b = p;
            break;
        case 1:
            r = q, g = v, b = p;
            break;
        case 2:
            r = p, g = v, b = t;
            break;
        case 3:
            r = p, g = q, b = v;
            break;
        case 4:
            r = t, g = p, b = v;
            break;
        case 5:
            r = v, g = p, b = q;
            break;
        default:
            r = v, g = p, b = q;
            break;
        }
    }

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

    void getTransformation(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t)
    {
        return pcl::getTransformation(x, y, z, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw), t);
    }

    bool getTransformation(const QString& text, Eigen::Affine3f& t)
    {
#if (QT_VERSION <= QT_VERSION_CHECK(5,14,0))
        QStringList valuesStr = text.simplified().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
#else
        QStringList valuesStr = text.simplified().split(QRegExp(",|\\s+"), Qt::SplitBehaviorFlags::SkipEmptyParts);
#endif
        if (valuesStr.size() == MATRIX_SIZE)
        {
            Eigen::Matrix4f mat;
            bool ok = false;
            for (int r = 0, idx = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                {
                    t.matrix()(r, c) = valuesStr[idx++].toFloat(&ok);
                    if (!ok) return false;
                }

            // internalRescale
            if (t.matrix()(3, 3) != 1 && t.matrix()(3, 3) != 0)
            {
                for (int r = 0; r < 4; r++)
                    for (int c = 0; c < 4; c++)
                        t.matrix()(r, c) *= 1.0 / t.matrix()(3, 3);
                t.matrix()(3, 3) = 1;
            }
        }
        else if (valuesStr.size() == EULER_SIZE)
        {
            bool ok = false;
            float val[6];
            for (int i = 0;i < 6;i++)
            {
                val[i] = valuesStr[0].toFloat(&ok);
                if (!ok) return false;
            }
            t = getTransformation(val[0], val[1], val[2], val[3], val[4], val[5]);
        }
        else return false;
        return true;
    }

    QString getTransformationQString(const Eigen::MatrixXf& mat, int decimals)
    {
        QString str;
        for (int i = 0; i < mat.rows(); i++)
        {
            for (int j = 0; j < mat.cols(); j++)
            {
                str += QString::number(mat(i, j), 'f', decimals) + " ";
            }
            if (i < mat.rows() - 1) str += "\n";
        }
        return str;
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

    Eigen::Matrix3f getRotationMatrix(float roll, float pitch, float yaw)
    {
        Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(pcl::deg2rad(roll), Eigen::Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(pcl::deg2rad(pitch), Eigen::Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(pcl::deg2rad(yaw), Eigen::Vector3f::UnitZ()));
        return Eigen::Matrix3f(yawAngle * pitchAngle * rollAngle);
    }



} // namespace ct
