#include "commands/Autos.h"
#include "commands/Score.h"
#include <frc2/command/Commands.h>

frc2::CommandPtr autos::ScoreL4LeftPole(Swerve &drive, ScoringMech &scoringMech, const choreo::Trajectory<choreo::SwerveSample> &trajectory1, const choreo::Trajectory<choreo::SwerveSample> &trajectory2) {
  return drive.setInitialTrajectoryCmd(trajectory1)
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Left(drive, scoringMech))
    .AndThen(drive.followTrajectory(trajectory2))
    .AndThen(scoringMech.home());
}

frc2::CommandPtr autos::ScoreL4RightPole(Swerve &drive, ScoringMech &scoringMech, const choreo::Trajectory<choreo::SwerveSample> &trajectory1, const choreo::Trajectory<choreo::SwerveSample> &trajectory2) {
  return drive.setInitialTrajectoryCmd(trajectory1)
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Right(drive, scoringMech))
    .AndThen(drive.followTrajectory(trajectory2))
    .AndThen(scoringMech.home());
}

frc2::CommandPtr autos::RightAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj3,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj4 ) {
  return drive.setInitialTrajectoryCmd(trajectory1)
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Right(drive, scoringMech))
    .AndThen(drive.followTrajectory(trajectory2))
    .AndThen(frc2::cmd::Parallel(
             frc2::cmd::Sequence(drive.followTrajectory(algaeTraj1),
                                 drive.followTrajectory(algaeTraj2)),
             scoringMech.intakeL3_5()))
    .AndThen(frc2::cmd::Parallel(drive.followTrajectory(algaeTraj3),
                                 scoringMech.goBarge()))
    .AndThen(scoringMech.ejectAlgae())
    .AndThen(frc2::cmd::Parallel(drive.followTrajectory(algaeTraj4),
                                scoringMech.home()).WithName("Right Algae Auto"));
}

frc2::CommandPtr autos::CenterAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj3,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj4 ) {
  return drive.setInitialTrajectoryCmd(trajectory1)
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Right(drive, scoringMech))
    .AndThen(drive.followTrajectory(trajectory2))
    .AndThen(frc2::cmd::Parallel(
             frc2::cmd::Sequence(drive.followTrajectory(algaeTraj1),
                                 drive.followTrajectory(algaeTraj2)),
             scoringMech.intakeL2_5()))
    .AndThen(frc2::cmd::Parallel(drive.followTrajectory(algaeTraj3),
                                 scoringMech.goBarge()))
    .AndThen(scoringMech.ejectAlgae())
    .AndThen(frc2::cmd::Parallel(drive.followTrajectory(algaeTraj4),
                                scoringMech.home()).WithName("Center Algae Auto"));
}

frc2::CommandPtr autos::LeftAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj3 ) {
  return drive.setInitialTrajectoryCmd(trajectory1)
    .AndThen(frc2::cmd::Parallel(
      drive.followTrajectory(trajectory1),
      scoringMech.goL4()))
    .AndThen(Score::Left(drive, scoringMech))
    .AndThen(drive.followTrajectory(trajectory2))
    .AndThen(frc2::cmd::Parallel(
             frc2::cmd::Sequence(drive.followTrajectory(algaeTraj1),
                                 drive.followTrajectory(algaeTraj2)),
             scoringMech.intakeL3_5()))
    .AndThen(frc2::cmd::Parallel(drive.followTrajectory(algaeTraj3),
                                 scoringMech.intake())).WithName("Left Algae Auto");
}