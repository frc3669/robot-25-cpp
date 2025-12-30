#include <frc2/command/Commands.h>
#include "commands/Score.h"

frc2::CommandPtr Score::Right(Swerve &swerve, ScoringMech &scoringMech) {
    return swerve.driveToRightPole()
        .AndThen(scoringMech.ejectCoral())
        .RaceWith(frc2::cmd::Wait(5_s))
        .OnlyIf([&swerve] { return swerve.reefWithinRange(); })
        .WithName("Score Right");
}

frc2::CommandPtr Score::Left(Swerve &swerve, ScoringMech &scoringMech) {
    return swerve.driveToLeftPole()
        .AndThen(scoringMech.ejectCoral())
        .RaceWith(frc2::cmd::Wait(5_s))
        .OnlyIf([&swerve] { return swerve.reefWithinRange(); })
        .WithName("Score Left");
}