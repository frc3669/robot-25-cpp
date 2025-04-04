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
    inline constexpr float max_current = 28;
    inline constexpr float feedforward_current = 4;
    inline constexpr float current_headroom = 0;
    inline constexpr float max_accel = 11.2;
    inline constexpr float max_m_per_sec_per_cycle = max_accel * MainConst::code_cycle_time;
    inline constexpr float current_to_accel_ratio = 9;
    inline constexpr float motor_turns_per_wheel_turn = 6.75;
    inline constexpr float wheel_diameter_m = 0.11266525; //0.10081
    inline constexpr float motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter_m*M_PI);
    inline constexpr float max_m_per_sec = 4.5;

    inline constexpr float position_P = 0.04;
    inline constexpr float heading_P = 2.5;

    inline constexpr float autoalign_P = 0.5;
}

namespace ScoreMechConst {
    inline constexpr float elevator_in_to_rotations = 2.54; // 9/9cm*2.54
    inline constexpr float angle_gear_ratio = 30;
    inline constexpr float algae_angle_gear_ratio = 54;
}