#pragma once
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <complex.h>
#include "angleMath.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/controller/PIDController.h"
#include "frc/controller/ProfiledPIDController.h"
#include "frc/controller/SimpleMotorFeedforward.h"

class SwerveModule {
  public:
    SwerveModule(int moduleID);
    void setVelocity(complex<float> robotVel, float angularVel, complex<float> robotAccel, float angularAccel);
    void brake();
    void resetEncoders();
    frc::SwerveModulePosition GetPosition();
    frc::Translation2d GetDeltaTranslation();
    void InitializeOdometry();
    void setDesiredStateTeleop(frc::SwerveModuleState & referenceState);
    void setDesiredStateAutonomous(frc::SwerveModuleState & referenceState, frc::SwerveModuleState & referenceAccelerationState);

  private:
    int m_moduleID;
    ctre::phoenix6::hardware::TalonFX m_driveMotor;
    ctre::phoenix6::hardware::TalonFX m_steeringMotor;
    ctre::phoenix6::hardware::CANcoder m_encoder;
	  ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocity{0_tps};
	  ctre::phoenix6::controls::TorqueCurrentFOC m_torque{0_A};
    units::meter_t lastWheelDistance = 0_m;
};