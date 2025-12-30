#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/Swerve.h"
#include "subsystems/ScoringMech.h"
#include "choreo/Choreo.h"

namespace autos {
    // follows first path while raising the elevator, scores to the left pole, then follows second path
    frc2::CommandPtr ScoreL4LeftPole(Swerve &drive, ScoringMech &scoringMech, const choreo::Trajectory<choreo::SwerveSample> &trajectory1, const choreo::Trajectory<choreo::SwerveSample> &trajectory2);
    // follows first path while raising the elevator, scores to the right pole, then follows second path
    frc2::CommandPtr ScoreL4RightPole(Swerve &drive, ScoringMech &scoringMech, const choreo::Trajectory<choreo::SwerveSample> &trajectory1, const choreo::Trajectory<choreo::SwerveSample> &trajectory2);
    frc2::CommandPtr RightAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj3,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj4 );
    frc2::CommandPtr CenterAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj3,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj4 );
    frc2::CommandPtr LeftAlgaeAuto(Swerve &drive, ScoringMech &scoringMech,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory1,
        const choreo::Trajectory<choreo::SwerveSample> &trajectory2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj1,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj2,
        const choreo::Trajectory<choreo::SwerveSample> &algaeTraj3 );
}
