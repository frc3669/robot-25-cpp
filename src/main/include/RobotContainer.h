// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/Joystick.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "subsystems/ScoringMech.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc::Joystick driverConroller{0};
  frc::Joystick xKeys{1};
  frc2::CommandJoystick cmdXKeys{1};
  
  // subsystems...
  ScoringMech scoringMech{&xKeys};

  void ConfigureBindings();
  void InitSubsystems();
};
