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
    .AndThen(drive.followTrajectory(&trajectory2));
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
    .AndThen(drive.followTrajectory(&trajectory2));
}