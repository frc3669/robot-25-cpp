// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/Trigger.h>
#include "commands/Score.h"
#include "commands/Autos.h"

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // scoring mechanism button bindings
  m_cmdXKeys.Button(9).OnTrue(m_scoringMech.intake());
  m_cmdXKeys.Button(17).OnTrue(m_scoringMech.home());
  m_cmdXKeys.Button(6).OnTrue(m_scoringMech.home());
  m_cmdXKeys.Button(16).OnTrue(m_scoringMech.coralReset());
  m_cmdXKeys.Button(15).OnTrue(m_scoringMech.goL2());
  m_cmdXKeys.Button(14).OnTrue(m_scoringMech.goL3());
  m_cmdXKeys.Button(13).OnTrue(m_scoringMech.goL4());
  m_cmdXKeys.Button(10).OnTrue(m_scoringMech.ejectCoral());
  m_cmdXKeys.Button(5).OnTrue(m_scoringMech.intakeAlgae());
  m_cmdXKeys.Button(3).OnTrue(m_scoringMech.intakeL3_5());
  m_cmdXKeys.Button(4).OnTrue(m_scoringMech.intakeL2_5());
  m_cmdXKeys.Button(1).OnTrue(m_scoringMech.scoreBarge());
  m_cmdXKeys.Button(2).OnTrue(m_scoringMech.scoreProcessor());
  m_cmdXKeys.Button(7).OnTrue(m_scoringMech.setHeightAndAnglesCmd(0,0,170));
  // climber control button bindings
  m_cmdXKeys.Button(18).WhileTrue(m_climber.extend());
  m_cmdXKeys.Button(19).WhileTrue(m_climber.retract());
  // autoscore button bindings
  m_cmdXKeys.Button(12).OnTrue(Score::Right(m_drive, m_scoringMech));
  m_cmdXKeys.Button(11).OnTrue(Score::Left(m_drive, m_scoringMech));
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
