/**
 * @file common.cpp
 * @author hjm (hjmalex@163.com)
 * @version 1.0
 * @date 2022-05-08
 */
#include "base/common.h"

namespace ct
{
  void HSVtoRGB(float h, float s, float v, float &r, float &g, float &b)
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
      case 0: r = v; g = t; b = p; break;
      case 1: r = q; g = v; b = p; break;
      case 2: r = p; g = v; b = t; break;
      case 3: r = p; g = q; b = v; break;
      case 4: r = t; g = p; b = v; break;
      case 5: r = v, g = p, b = q; break;
      default:r = v; g = p; b = q; break;
    }
  }

  
} // namespace ct