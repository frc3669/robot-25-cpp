#include "angleMath.h"

void am::limit(float &angle) {
    while (angle > M_PI){
        angle -= M_PI*2;
    }
    while (angle < -M_PI){
        angle += M_PI*2;
    }
}

void am::limitDeg(float &angle) {
    while (angle > 180){
        angle -= 360;
    }
    while (angle < -180){
        angle += 360;
    }
}

float am::getProjectionSize(complex<float> a, complex<float> b)  {
    if (abs(b) != 0)
        b /= abs(b);
    return a.real()*b.real() + a.imag()*b.imag();
}