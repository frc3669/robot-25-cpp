#include "angleMath.h"

void am::limit(float &angle) {
    while (angle > M_PI){
        angle -= M_PI*2;
    }
    while (angle < -M_PI){
        angle += M_PI*2;
    }
}

void am::limit(double &angle) {
    while (angle > M_PI){
        angle -= M_PI*2;
    }
    while (angle < -M_PI){
        angle += M_PI*2;
    }
}

void am::limit(units::radian_t &angle) {
    while (angle > 180_deg) {
        angle -= 360_deg;
    }
    while (angle < -180_deg) {
        angle += 360_deg;
    }
}

void am::limit(units::degree_t &angle) {
    while (angle > 180_deg) {
        angle -= 360_deg;
    }
    while (angle < -180_deg) {
        angle += 360_deg;
    }
}

float am::getProjectionSize(complex<float> a, complex<float> b)  {
    if (abs(b) != 0)
        b /= abs(b);
    return a.real()*b.real() + a.imag()*b.imag();
}