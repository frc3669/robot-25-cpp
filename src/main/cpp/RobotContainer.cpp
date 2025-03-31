// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/Trigger.h>
#include "commands/Score.h"
#include "commands/Autos.h"

RobotContainer::RobotContainer() {
  SwerveModule module1{1, 1, 1};
  SwerveModule module2{2, -1, 1};
  SwerveModule module3{3, -1, -1};
  SwerveModule module4{4, 1, -1};
  m_drive.addModule(module1);
  m_drive.addModule(module2);
  m_drive.addModule(module3);
  m_drive.addModule(module4);

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // scoring mechanism button bindings
  m_XKeys.Button(9).OnTrue(m_scoringMech.intake());
  m_XKeys.Button(17).OnTrue(m_scoringMech.home());
  m_XKeys.Button(6).OnTrue(m_scoringMech.home());
  m_XKeys.Button(16).OnTrue(m_scoringMech.coralReset());
  m_XKeys.Button(15).OnTrue(m_scoringMech.goL2());
  m_XKeys.Button(14).OnTrue(m_scoringMech.goL3());
  m_XKeys.Button(13).OnTrue(m_scoringMech.goL4());
  m_XKeys.Button(10).OnTrue(m_scoringMech.ejectCoral());
  m_XKeys.Button(5).OnTrue(m_scoringMech.intakeAlgae());
  m_XKeys.Button(3).OnTrue(m_scoringMech.intakeL3_5());
  m_XKeys.Button(4).OnTrue(m_scoringMech.intakeL2_5());
  m_XKeys.Button(1).OnTrue(m_scoringMech.scoreBarge());
  m_XKeys.Button(2).OnTrue(m_scoringMech.scoreProcessor());
  m_XKeys.Button(7).OnTrue(m_scoringMech.prepareForClimb());
  // climber control button bindings
  m_XKeys.Button(18).WhileTrue(m_climber.extend());
  m_XKeys.Button(19).WhileTrue(m_climber.retract());
  // autoscore button bindings
  m_XKeys.Button(12).OnTrue(Score::Right(m_drive, m_scoringMech));
  m_XKeys.Button(11).OnTrue(Score::Left(m_drive, m_scoringMech));
}

void RobotContainer::ConfigureChooser() {
  m_chooser.SetDefaultOption("Center Auto", m_centerAuto.get());
  m_chooser.AddOption("Left Auto", m_leftAuto.get());
  m_chooser.AddOption("Right Auto", m_rightAuto.get());
}

void RobotContainer::ConfigureDefaultCommands() {
  m_drive.SetDefaultCommand(std::move(m_drive.defaultDrive()));
  m_climber.SetDefaultCommand(std::move(m_climber.brake()));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}

void RobotContainer::DisplaySchedulerDetails() {
  frc::SmartDashboard::PutData("Command Scheduler Status", &frc2::CommandScheduler::GetInstance());
  frc::SmartDashboard::PutData("Swerve Status", &m_drive);
  frc::SmartDashboard::PutData("Scoring Mechanism Status", &m_scoringMech);
  frc::SmartDashboard::PutData("Climb Status", &m_climber);
}