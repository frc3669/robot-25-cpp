#include "commands/Autos.h"
#include "commands/Score.h"
#include <frc2/command/Commands.h>

frc2::CommandPtr autos::ScoreL4LeftPole(Swerve &drive, ScoringMech &scoringMech, choreo::Trajectory<choreo::SwerveSample> &trajectory1, choreo::Trajectory<choreo::SwerveSample> &trajectory2) {
  auto initialPose = trajectory1.GetInitialPose().value();
  auto traj2InitialPose = trajectory2.GetInitialPose().value();
  return drive.resetPoseCmd(complex<float>(initialPose.X().value(), initialPose.Y().value()), initialPose.Rotation().Radians().value())
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(&trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Left(drive, scoringMech))
    .AndThen(drive.resetPositionCmd(complex<float>(traj2InitialPose.X().value(), traj2InitialPose.Y().value())))
    .AndThen(drive.followTrajectory(&trajectory2))
    .AndThen(scoringMech.home());
}

frc2::CommandPtr autos::ScoreL4RightPole(Swerve &drive, ScoringMech &scoringMech, choreo::Trajectory<choreo::SwerveSample> &trajectory1, choreo::Trajectory<choreo::SwerveSample> &trajectory2) {
  auto initialPose = trajectory1.GetInitialPose().value();
  auto traj2InitialPose = trajectory2.GetInitialPose().value();
  return drive.resetPoseCmd(complex<float>(initialPose.X().value(), initialPose.Y().value()), initialPose.Rotation().Radians().value())
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(&trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Right(drive, scoringMech))
    .AndThen(drive.resetPositionCmd(complex<float>(traj2InitialPose.X().value(), traj2InitialPose.Y().value())))
    .AndThen(drive.followTrajectory(&trajectory2))
    
    .AndThen(scoringMech.home());
}

frc2::CommandPtr autos::RightAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        choreo::Trajectory<choreo::SwerveSample> &algaeTraj3 ) {
  auto initialPose = trajectory1.GetInitialPose().value();
  auto traj2InitialPose = trajectory2.GetInitialPose().value();
  return drive.resetPoseCmd(complex<float>(initialPose.X().value(), initialPose.Y().value()), initialPose.Rotation().Radians().value())
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(&trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Right(drive, scoringMech))
    .AndThen(drive.resetPositionCmd(complex<float>(traj2InitialPose.X().value(), traj2InitialPose.Y().value())))
    .AndThen(drive.followTrajectory(&trajectory2))
    .AndThen(frc2::cmd::Parallel(
             frc2::cmd::Sequence(drive.followTrajectory(&algaeTraj1),
                                 drive.followTrajectory(&algaeTraj2)),
             scoringMech.intakeL3_5()))
    .AndThen(frc2::cmd::Parallel(drive.followTrajectory(&algaeTraj3),
                                 scoringMech.goBarge()))
    .AndThen(scoringMech.ejectAlgae())
    .AndThen(scoringMech.home());
  
}

frc2::CommandPtr autos::CenterAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        choreo::Trajectory<choreo::SwerveSample> &algaeTraj3 ) {
  auto initialPose = trajectory1.GetInitialPose().value();
  auto traj2InitialPose = trajectory2.GetInitialPose().value();
  return drive.resetPoseCmd(complex<float>(initialPose.X().value(), initialPose.Y().value()), initialPose.Rotation().Radians().value())
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(&trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Right(drive, scoringMech))
    .AndThen(drive.resetPositionCmd(complex<float>(traj2InitialPose.X().value(), traj2InitialPose.Y().value())))
    .AndThen(drive.followTrajectory(&trajectory2))
    .AndThen(frc2::cmd::Parallel(
             frc2::cmd::Sequence(drive.followTrajectory(&algaeTraj1),
                                 drive.followTrajectory(&algaeTraj2)),
             scoringMech.intakeL2_5()))
    .AndThen(frc2::cmd::Parallel(drive.followTrajectory(&algaeTraj3),
                                 scoringMech.goBarge()))
    .AndThen(scoringMech.ejectAlgae())
    .AndThen(scoringMech.home());
  
}