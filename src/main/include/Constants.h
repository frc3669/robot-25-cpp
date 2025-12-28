#pragma once
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/time.h>
// constants for the robot as a whole
namespace MainConst {
    inline constexpr float code_cycle_time = 0.02;
}

namespace DriverControllerConstants {
    inline constexpr double dB = 0.1;
}

// constants for the swerve
namespace SwerveConstants {
    inline constexpr double max_current = 100;
    inline constexpr double feedforward_current = 4;
    inline constexpr double current_headroom = 4;
    inline constexpr double max_accel = 8;
    inline constexpr double braking_accel = 4;
    inline constexpr double max_m_per_sec_per_cycle = max_accel * MainConst::code_cycle_time;
    inline constexpr units::time::second_t time_to_full_speed = 0.5625_s;
    inline constexpr double current_to_accel_ratio = 9;
    inline constexpr double motor_turns_per_wheel_turn = 5.9;
    inline constexpr double wheel_diameter_m = 0.10081;
    inline constexpr double motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter_m*M_PI);
    inline constexpr units::velocity::meters_per_second_t max_m_per_sec = 4.5_mps;
    inline constexpr units::angular_velocity::radians_per_second_t max_rad_per_sec = 20.88_rad_per_s;

    inline constexpr double position_P = 0.04;
    inline constexpr double heading_P = 1.5;

    inline constexpr double autoalign_P = 0.3;
}

namespace ScoreMechConst {
    inline constexpr double elevator_in_to_rotations = 5.0/9*2.54;
    inline constexpr double angle_gear_ratio = 30;
    inline constexpr double algae_angle_gear_ratio = 54;
}