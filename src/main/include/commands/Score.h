#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/ScoringMech.h"
#include "subsystems/Swerve.h"

namespace Score {
    frc2::CommandPtr Right(Swerve &swerve, ScoringMech &scoringMech);
    frc2::CommandPtr Left(Swerve &swerve, ScoringMech &scoringMech);
}