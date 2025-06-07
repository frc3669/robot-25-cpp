#pragma once
// constants for the robot as a whole
namespace MainConst {
    inline constexpr float code_cycle_time = 0.02;
}

namespace DriverControllerConstants {
    inline constexpr float dB = 0.1;
}

// constants for the swerve
namespace SwerveConstants {
    inline constexpr float max_current = 100;
    inline constexpr float feedforward_current = 4;
    inline constexpr float current_headroom = 4;
    inline constexpr float max_accel = 10;
    inline constexpr float braking_accel = 4;
    inline constexpr float max_m_per_sec_per_cycle = max_accel * MainConst::code_cycle_time;
    inline constexpr float current_to_accel_ratio = 9;
    inline constexpr float motor_turns_per_wheel_turn = 5.9;
    inline constexpr float wheel_diameter_m = 0.10081; //0.11266525
    inline constexpr float motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter_m*M_PI);
    inline constexpr float max_m_per_sec = 4.5;

    inline constexpr float position_P = 0.04;
    inline constexpr float heading_P = 2.5;

    inline constexpr float autoalign_P = 0.3;
}

namespace ScoreMechConst {
    inline constexpr float elevator_in_to_rotations = 5.0/9*2.54;
    inline constexpr float angle_gear_ratio = 30;
    inline constexpr float algae_angle_gear_ratio = 54;
}