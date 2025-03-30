#include <iostream>
#include <util.h>

using namespace ctre::phoenix6;

void Util::configureMotor(hardware::TalonFX *motor, configs::TalonFXConfiguration *config) {
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = motor->GetConfigurator().Apply(*config);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    }
}

void Util::configureMotor(hardware::TalonFXS *motor, configs::TalonFXSConfiguration *config) {
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = motor->GetConfigurator().Apply(*config);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    }
}