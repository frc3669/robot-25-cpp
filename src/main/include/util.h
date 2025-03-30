#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>

namespace Util {
    void configureMotor(ctre::phoenix6::hardware::TalonFX *motor, ctre::phoenix6::configs::TalonFXConfiguration *config);
    void configureMotor(ctre::phoenix6::hardware::TalonFXS *motor, ctre::phoenix6::configs::TalonFXSConfiguration *config);
}