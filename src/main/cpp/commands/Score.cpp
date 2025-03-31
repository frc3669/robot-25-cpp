#include <frc2/command/Commands.h>
#include "commands/Score.h"

frc2::CommandPtr Score::Right(Swerve &swerve, ScoringMech &scoringMech) {
    return swerve.driveRightToPole()
        .AndThen(scoringMech.ejectCoral())
        .RaceWith(frc2::cmd::Wait(3_s))
        .WithName("Score Right");
}

frc2::CommandPtr Score::Left(Swerve &swerve, ScoringMech &scoringMech) {
    return swerve.driveLeftToPole()
        .AndThen(scoringMech.ejectCoral())
        .RaceWith(frc2::cmd::Wait(3_s))
        .WithName("Score Left");
}