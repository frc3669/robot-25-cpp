#pragma once
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <complex.h>
#include "angleMath.h"

class SwerveModule {
  public:
    SwerveModule(int moduleID, float modulePositionX, float modulePositionY);
    void setVelocity(complex<float> robotVel, float angularVel, complex<float> robotAccel, float angularAccel);
    void brake();
    complex<float> findModuleVector(complex<float> robotVec, float angularVec);
    float getAccelOvershoot(complex<float> robotVel, float angularVel, complex<float> robotVelIncrement, float angularVelIncrement);
    complex<float> getPositionChange();
    void resetEncoders();
    void init();

  private:
    void odometryCalc();
    float getAccelCurrent(complex<float> accelCurrentVector, float wheelAngle);
    ctre::phoenix6::hardware::TalonFX *dMotor;
    ctre::phoenix6::hardware::TalonFX *sMotor;
    ctre::phoenix6::hardware::CANcoder *encoder;
	ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocity{0_tps};
    complex<float> turnVector;
    complex<float> posChg = complex<float>(0,0);
    float motorPosOld = 0;
    float angle = 0;
    int moduleID;
};