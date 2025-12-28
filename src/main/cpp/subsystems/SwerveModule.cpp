#include "subsystems/SwerveModule.h"
#include "Constants.h"
#include "angleMath.h"
#include "util.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/filter/SlewRateLimiter.h>
#include <string>

using namespace ctre::phoenix6;
using namespace std;

// create a Swerve module object with specified position and ID
SwerveModule::SwerveModule(int moduleID) : 
        m_moduleID(moduleID), m_driveMotor(10 + moduleID, "CTREdevices"),
        m_steeringMotor(20 + moduleID, "CTREdevices"),
        m_encoder(30 + moduleID, "CTREdevices") {
    m_driveMotorTurns = new StatusSignal(m_driveMotor.GetPosition());
    m_encoderTurns = new StatusSignal(m_encoder.GetAbsolutePosition());
    configs::TalonFXConfiguration cfg{};
    cfg.Slot0.kP = 5;
    cfg.Slot0.kS = 3;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants::max_current * 1_A;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants::max_current * 1_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 20_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 0_s;
    cfg.CurrentLimits.SupplyCurrentLimit = 20_A;
    cfg.CurrentLimits.StatorCurrentLimit = 80_A;
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    Util::configureMotor(m_driveMotor, cfg);
    Util::configureMotor(m_steeringMotor, cfg);
}

void SwerveModule::setDesiredStateAutonomous(frc::SwerveModuleState & referenceState, frc::SwerveModuleState & referenceAccelerationState) {
    units::degree_t encoderAngle{m_encoderTurns->GetValue()};
    auto targetAngle = referenceState.angle.Degrees();
    auto angleError = targetAngle-encoderAngle;
    am::limit(angleError);
    // simultaneously optimize module direction and reduce module speed when pointed in the wrong direction
    auto moduleSpeed = referenceState.speed*units::math::cos(angleError);
    if (units::math::abs(angleError) > 90_deg) {
        angleError += 180_deg;
        am::limit(angleError);
    }
    m_steeringMotor.SetControl(controls::DutyCycleOut(angleError/180_deg));
    m_driveMotor.SetControl(m_velocity
        .WithVelocity(moduleSpeed.value()*SwerveConstants::motor_turns_per_m*1_tps));
    // TODO: add working acceleration feedforward and use this method
}

void SwerveModule::setDesiredStateTeleop(frc::SwerveModuleState & referenceState) {
    
    units::degree_t encoderAngle{m_encoderTurns->GetValue()};
    auto targetAngle = referenceState.angle.Degrees();
    auto angleError = targetAngle-encoderAngle;
    am::limit(angleError);
    // simultaneously optimize module direction and reduce module speed when pointed in the wrong direction
    auto moduleSpeed = referenceState.speed*units::math::cos(angleError);
    if (units::math::abs(angleError) > 90_deg) {
        angleError += 180_deg;
        am::limit(angleError);
    }
    m_steeringMotor.SetControl(controls::DutyCycleOut(angleError/180_deg));
    m_driveMotor.SetControl(m_velocity
        .WithVelocity(moduleSpeed.value()*SwerveConstants::motor_turns_per_m*1_tps));
}

// set the drive motor to brake mode
void SwerveModule::brake() {
    m_driveMotor.SetControl(controls::NeutralOut());
    m_steeringMotor.SetControl(controls::StaticBrake());
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {units::meter_t{m_driveMotorTurns->GetValueAsDouble()/SwerveConstants::motor_turns_per_m},
            units::radian_t{m_encoderTurns->GetValueAsDouble()}};
}

void SwerveModule::InitializeOdometry() {
    lastWheelDistance = units::meter_t{m_driveMotorTurns->GetValueAsDouble() / SwerveConstants::motor_turns_per_m};
}

frc::Translation2d SwerveModule::GetDeltaTranslation() {
    auto angle = m_encoderTurns->GetValue();
    auto wheelDistance = units::meter_t{m_driveMotorTurns->GetValueAsDouble() / SwerveConstants::motor_turns_per_m};
    auto deltaPosition = wheelDistance - lastWheelDistance;
    lastWheelDistance = wheelDistance;
    return {deltaPosition*units::math::cos(angle), deltaPosition*units::math::sin(angle)};
}

SwerveModule::~SwerveModule() {
    delete m_driveMotorTurns;
    delete m_encoderTurns;
}