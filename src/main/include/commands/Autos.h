#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/Swerve.h"
#include "subsystems/ScoringMech.h"
#include "choreo/Choreo.h"

namespace autos {
    // follows first path while raising the elevator, scores to the left pole, then follows second path
    frc2::CommandPtr ScoreL4LeftPole(Swerve &drive, ScoringMech &scoringMech, choreo::Trajectory<choreo::SwerveSample> &trajectory1, choreo::Trajectory<choreo::SwerveSample> &trajectory2);
    // follows first path while raising the elevator, scores to the right pole, then follows second path
    frc2::CommandPtr ScoreL4RightPole(Swerve &drive, ScoringMech &scoringMech, choreo::Trajectory<choreo::SwerveSample> &trajectory1, choreo::Trajectory<choreo::SwerveSample> &trajectory2);
}  // namespace autos
