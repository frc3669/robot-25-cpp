#pragma once
#include <math.h>
#include <complex.h>

using namespace std;

namespace am{
    void limit(float &angle);
    void limitDeg(float &angle);
    float getProjectionSize(complex<float> a, complex<float> b);
}