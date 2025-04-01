#include "subsystems/SwerveModule.h"
#include "Constants.h"
#include "angleMath.h"
#include "util.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "string"

using namespace ctre::phoenix6;
using namespace std;

// create a Swerve module object with specified position and ID
SwerveModule::SwerveModule(int moduleID, float modulePositionX, float modulePositionY) {
    turnVector = complex<float>(modulePositionX, modulePositionY) * complex<float>(0,1);
    if(abs(turnVector) != 0) {
        turnVector /= abs(turnVector);
    }
    this->moduleID = moduleID;
    dMotor = new hardware::TalonFX(10 + moduleID, "CTREdevices");
    sMotor = new hardware::TalonFX(20 + moduleID, "CTREdevices");
    encoder = new hardware::CANcoder(30 + moduleID, "CTREdevices");
    configs::TalonFXConfiguration cfg{};
    cfg.Slot0.kP = 5;
    cfg.Slot0.kS = 3;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants::max_current * 1_A;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants::max_current * 1_A;
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    Util::configureMotor(dMotor, &cfg);
}

// calculates the position change of this module
void SwerveModule::odometryCalc(){
    angle = encoder->GetAbsolutePosition().GetValueAsDouble() * M_PI * 2;
    float motorPos = dMotor->GetPosition().GetValueAsDouble();
    float motorPosChg = motorPos - motorPosOld;
    motorPosOld = motorPos;
    posChg = polar<float>(motorPosChg/SwerveConstants::motor_turns_per_m, angle);
}

// drive the wheel in a direction with acceleration feedforward
void SwerveModule::setVelocity(complex<float> robotVel, float angularVel, complex<float> robotAccel, float angularAccel) {
    frc::SmartDashboard::PutNumber("module"+std::to_string(moduleID)+" angle", angle);
    complex<float> vel = findModuleVector(robotVel, angularVel);
    complex<float> accelCurrentVec = findModuleVector(robotAccel, angularAccel);
    float wheelSpeed = abs(vel);
    odometryCalc();
    float error = arg(vel) - angle;
    am::limit(error);
    if (wheelSpeed < 0.008)
        error = 0;
    if (abs(error) > M_PI/2) {
        error += M_PI;
        am::limit(error);
        wheelSpeed *= -1;
    }
    sMotor->Set(error/M_PI);
    float wheelAccelCurrent = getAccelCurrent(accelCurrentVec, angle);
    dMotor->SetControl(m_velocity
        .WithVelocity(wheelSpeed*SwerveConstants::motor_turns_per_m * 1_tps)
        .WithFeedForward(wheelAccelCurrent*1_A));
}

// get the torque current to apply to the motor based on an acceleration vector
float SwerveModule::getAccelCurrent(complex<float> accelCurrentVector, float wheelAngle) {
    complex<float> wheelUnitVector = polar<float>(1, wheelAngle);
    return accelCurrentVector.real()*wheelUnitVector.real() + accelCurrentVector.imag()*wheelUnitVector.imag();
}

// set the drive motor to brake mode
void SwerveModule::brake() {
    dMotor->SetControl(controls::NeutralOut());
    sMotor->SetControl(controls::StaticBrake());
}

/**
 * return the vector for this module given the robot-oriented
 * velocity and a rotation rate
**/
complex<float> SwerveModule::findModuleVector(complex<float> robotVec, float angularVec) {
    return robotVec + turnVector*angularVec;
}

// find the acceleration overshoot of this module for normalization
float SwerveModule::getAccelOvershoot(complex<float> robotVel, float angularVel, complex<float> robotVelIncrement, float angularVelIncrement){
    complex<float> vel = findModuleVector(robotVel, angularVel);
    complex<float> velIncrement = findModuleVector(robotVelIncrement, angularVelIncrement);
    float accelOvershoot = 1;
    if (abs(velIncrement) > SwerveConstants::max_m_per_sec_per_cycle) {
        accelOvershoot = abs(velIncrement) / SwerveConstants::max_m_per_sec_per_cycle;
    }
    float wheelCurrent = am::getProjectionSize(velIncrement/MainConst::code_cycle_time*SwerveConstants::current_to_accel_ratio, vel) + SwerveConstants::feedforward_current;
    float wheelAccelOvershoot = abs(wheelCurrent) / (SwerveConstants::max_current - SwerveConstants::current_headroom);
    if (wheelAccelOvershoot > accelOvershoot) {
        accelOvershoot = wheelAccelOvershoot;
    }
    return accelOvershoot;
}

// get the position change of this module since the last cycle
complex<float> SwerveModule::getPositionChange(){
    return posChg;
}

// reset the encoders
void SwerveModule::resetEncoders(){
    motorPosOld = 0;
    dMotor->SetPosition(0_tr);
}