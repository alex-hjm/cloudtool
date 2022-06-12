/**
 * @file common.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/common.h"

#include <QStringList>

#define MATRIX_SIZE 16

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

    bool getTransformation(const QString& matrix, Eigen::Affine3f& affine)
    {
        QStringList valuesStr = matrix.simplified().split(QChar(' '), Qt::SplitBehaviorFlags::SkipEmptyParts);
        if (valuesStr.size() != MATRIX_SIZE) return false;

        Eigen::Matrix4f mat;
        bool ok = false;
        for (int i = 0; i < MATRIX_SIZE; ++i)
        {
            mat(i / 4, i % 4) = (valuesStr[(i % 4) * 4 + (i >> 2)].toDouble(&ok));
            if (!ok) return false;
        }

        // internalRescale
        if (mat(3, 3) != 1 && mat(3, 3) != 0)
        {
            for (int i = 0;i < MATRIX_SIZE;i++) mat(i / 4, i % 4) *= 1.0 / mat(3, 3);
            mat(3, 3) = 1;
        }
        affine = Eigen::Affine3f(mat);
        return true;
    }


} // namespace ct
