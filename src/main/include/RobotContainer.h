// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "subsystems/ScoringMech.h"
#include "subsystems/Swerve.h"
#include "subsystems/Climb.h"
#include "commands/Autos.h"

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

  frc2::Command* GetAutonomousCommand();
  void ConfigureChooser();
  void ConfigureDefaultCommands();
  void DisplaySchedulerDetails();
  
  ScoringMech m_scoringMech{&m_XKeys};

 private:
  frc::GenericHID m_driverController{0};
  frc2::CommandGenericHID m_XKeys{1};
  
  // subsystems...
  Swerve m_drive{&m_driverController};
  Climb m_climber{};


  // autonomous trajectories
  choreo::Trajectory<choreo::SwerveSample> centerTraj1 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Center Path 1").value();
  choreo::Trajectory<choreo::SwerveSample> centerTraj2 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Center Path 2").value();
  choreo::Trajectory<choreo::SwerveSample> leftTraj1 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Left Path 1").value();
  choreo::Trajectory<choreo::SwerveSample> leftTraj2 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Left Path 2").value();
  choreo::Trajectory<choreo::SwerveSample> rightTraj1 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Right Path 1").value();
  choreo::Trajectory<choreo::SwerveSample> rightTraj2 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Right Path 2").value();

  // autonomous routines
  frc2::CommandPtr m_centerAuto = autos::ScoreL4RightPole(m_drive, m_scoringMech, centerTraj1, centerTraj2);
  frc2::CommandPtr m_leftAuto = autos::ScoreL4LeftPole(m_drive, m_scoringMech, leftTraj1, leftTraj2);
  frc2::CommandPtr m_rightAuto = autos::ScoreL4RightPole(m_drive, m_scoringMech, rightTraj1, rightTraj2);

  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureBindings();
};
