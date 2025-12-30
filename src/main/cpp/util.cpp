#include <iostream>
#include <util.h>
#include <math.h>
#include "Constants.h"

using namespace ctre::phoenix6;

void Util::configureMotor(hardware::TalonFX & motor, configs::TalonFXConfiguration const & config) {
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = motor.GetConfigurator().Apply(config);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    }
}

void Util::configureMotor(hardware::TalonFXS & motor, configs::TalonFXSConfiguration const & config) {
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = motor.GetConfigurator().Apply(config);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    }
}

double Util::desaturateChassisSpeeds(frc::ChassisSpeeds & robotSpeeds, wpi::array<frc::SwerveModuleState, 4U> const & states) {
    double fastestModuleSpeed = SwerveConstants::max_m_per_sec.value();
    for (auto & state : states) {
        if (state.speed.value() > fastestModuleSpeed) {
            fastestModuleSpeed = state.speed.value();
        }
    }
    double desaturationValue = SwerveConstants::max_m_per_sec.value() / fastestModuleSpeed;
    robotSpeeds = robotSpeeds * desaturationValue;
    return desaturationValue;
}

Util::SlewLimiter::SlewLimiter() {
    cycleTimer.Start();
}

void Util::SlewLimiter::Reset() {
    cycleTimer.Reset();
    slewSpeeds = frc::ChassisSpeeds{};
}

frc::ChassisSpeeds Util::SlewLimiter::GetSpeeds() {
    return slewSpeeds;
}

void Util::SlewLimiter::Run(const frc::ChassisSpeeds &targetSpeeds, const units::time::second_t &secondsToFullSpeed, const units::time::second_t &period) {
    double distance = sqrt(pow(targetSpeeds.vx.value()-slewSpeeds.vx.value(), 2)+pow(targetSpeeds.vy.value()-slewSpeeds.vy.value(), 2)+pow(targetSpeeds.omega.value()-slewSpeeds.omega.value(), 2));
    double incrementSize = SwerveConstants::max_m_per_sec.value() * period / secondsToFullSpeed;
    if (distance < incrementSize*2) {
        slewSpeeds = targetSpeeds;
    } else {
        slewSpeeds = slewSpeeds + (targetSpeeds-slewSpeeds) * (incrementSize/distance);
    }
}