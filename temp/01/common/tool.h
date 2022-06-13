#ifndef TOOL_H
#define TOOL_H
#include <QString>
#include <pcl/common/transforms.h>

class Tool
{
public:
    static Eigen::Affine3f AffineFromXYZEuler(float x, float y, float z, float roll, float pitch, float yaw)
    {
        Eigen::Affine3f rolation=Eigen::Affine3f::Identity();
        rolation=pcl::getTransformation(x,y,z,roll/180*M_PI,pitch/180*M_PI,yaw/180*M_PI);
        return rolation;
    }

    static Eigen::Affine3f AffineFromAxisAngle(float angle, float axisX, float axisY,float axisZ,
                                               float transX, float transY, float transZ)
    {
        float degree=angle/180*M_PI;
        Eigen::AngleAxisf rotation_vector(degree,Eigen::Vector3f(axisX,axisY,axisZ));
        Eigen::Vector3f eulerAngle=rotation_vector.matrix().eulerAngles(0,1,2);//r,p,y
        Eigen::Affine3f rolation=Eigen::Affine3f::Identity();
        rolation=pcl::getTransformation(transX,transY,transZ,eulerAngle[0],eulerAngle[1],eulerAngle[2]);
        return rolation;
    }

    static Eigen::Affine3f AffineFromQString(const QString &text,bool &success)
    {
        Eigen::Matrix4f Martrix=Eigen::Matrix4f::Identity();
        Eigen::Transform<float, 3, Eigen::Affine> a3f_transform (Martrix);
        a3f_transform.setIdentity();
        QStringList textlist = text.split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
        if (textlist.size() != 16){success = false;return a3f_transform;}
        for (int r = 0, idx = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                Martrix(r,c) = textlist[idx++].toFloat();
        success = true;
        a3f_transform.matrix()=Martrix;
        return a3f_transform;
    }

    static Eigen::Matrix3f RotMatrixFromEulerAngle(float roll, float pitch, float yaw)
    {
        Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(roll/180*M_PI,Eigen::Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(pitch/180*M_PI,Eigen::Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(yaw/180*M_PI,Eigen::Vector3f::UnitZ()));
        Eigen::Matrix3f rolation;
        rolation=yawAngle*pitchAngle*rollAngle;
        return rolation;
    }

    static Eigen::Matrix4f MatrixFromXYZEulerAngle(float x, float y, float z,float roll, float pitch, float yaw)
    {
        Eigen::Affine3f rolation=Tool::AffineFromXYZEuler(x,y,z,roll,pitch,yaw);
        return rolation.matrix();
    }

    static  Eigen::Matrix4f MatrixFromXYZRotMatrix(float x, float y, float z, Eigen::Matrix3f &rolation)
    {
        Eigen::Matrix4f Martrix=Eigen::Matrix4f::Identity();
        Martrix.topLeftCorner(3, 3)<<rolation;
        Martrix.topRightCorner(3, 1)<<x,y,z;
        Martrix(3, 0) = 0;
        Martrix(3, 1) = 0;
        Martrix(3, 2) = 0;
        Martrix(3, 3) = 1;
        return Martrix;
    }

    static Eigen::Matrix4f MatrixFromQString(const QString &text,bool &success)
    {
        Eigen::Affine3f rolation=Tool::AffineFromQString(text,success);
        return rolation.matrix();
    }

    static QString QStringFromMatrix(Eigen::MatrixXf Martrix,int decimals)
    {
        QString MartrixtoQString;
        for (int i=0;i<Martrix.rows();i++) {
            for (int j=0;j<Martrix.cols();j++)
            {
                MartrixtoQString+=QString::number(Martrix(i,j),'f',decimals)+" ";
            }
            if(i<Martrix.rows()-1)
                MartrixtoQString+="\n";
        }
        return MartrixtoQString;
    }

    static void ConvertHSVtoRGB(float h, float s, float v, float& out_r, float& out_g, float& out_b)
    {
        if (s == 0.0f)
        {
            out_r = out_g = out_b = v;
            return;
        }

        h = fmodf(h, 1.0f) / (60.0f / 360.0f);
        int   i = (int)h;
        float f = h - (float)i;
        float p = v * (1.0f - s);
        float q = v * (1.0f - s * f);
        float t = v * (1.0f - s * (1.0f - f));

        switch (i)
        {
        case 0: out_r = v; out_g = t; out_b = p; break;
        case 1: out_r = q; out_g = v; out_b = p; break;
        case 2: out_r = p; out_g = v; out_b = t; break;
        case 3: out_r = p; out_g = q; out_b = v; break;
        case 4: out_r = t; out_g = p; out_b = v; break;
        case 5: default: out_r = v; out_g = p; out_b = q; break;
        }
    }

};

#endif // TOOL_H
