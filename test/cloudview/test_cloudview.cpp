/**
 * @file test_cloudview.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-30
 */
#include <QApplication>

#include "base/cloudview.h"

ct::Cloud::Ptr create_cloud()
{
  ct::Cloud::Ptr cloud(new ct::Cloud);

  float radius = 0.1;
  int num_theta = 50;
  int num_phi = 25;

  for (int i = 0; i < num_phi; ++i) 
  {
    for (int j = 0; j < num_theta; ++j) 
    {
      float theta = static_cast<float>(j) / static_cast<float>(num_theta - 1) * 2.0 * M_PI;
      float phi = static_cast<float>(i) / static_cast<float>(num_phi - 1) * M_PI;

      ct::PointXYZRGBN point;
      point.x = radius * sin(phi) * cos(theta);
      point.y = radius * sin(phi) * sin(theta);
      point.z = radius * cos(phi);

      uint8_t r = static_cast<uint8_t>((cos(phi) + 1.0) * 0.5 * 255);
      uint8_t g = static_cast<uint8_t>((sin(phi) * cos(theta) + 1.0) * 0.5 * 255);
      uint8_t b = static_cast<uint8_t>((sin(phi) * sin(theta) + 1.0) * 0.5 * 255);

      point.r = r;
      point.g = g;
      point.b = b;

      cloud->points.push_back(point);
    }
  }

  return cloud;
}

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  ct::Cloud::Ptr cloud = create_cloud();
  ct::CloudView cv;
  cv.addCloud(cloud);
  
  cv.show();
  return a.exec();
}