#include "subsystems/Swerve.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <thread>

using namespace SwerveConstants;
using namespace MainConst;
using namespace DriverControllerConstants;
using namespace ctre::phoenix6;


Swerve::Swerve(int driverControllerPortNum) : 
        m_driverController(driverControllerPortNum) {
    // add all the status signals to a list for syncronized updates
    m_gyroAngleSignal = new StatusSignal(gyro.GetYaw());
    m_statusSignals.push_back(m_gyroAngleSignal);
    for (auto & module : m_moduleList) {
        m_statusSignals.push_back(module->m_driveMotorTurns);
        m_statusSignals.push_back(module->m_encoderTurns);
    }
    BaseStatusSignal::SetUpdateFrequencyForAll(200_Hz, m_statusSignals);
}

void Swerve::SimulationPeriodic() {}

void Swerve::Periodic() {
    frc::SmartDashboard::PutNumber("pose X", m_pose.X().value());
    frc::SmartDashboard::PutNumber("pose Y", m_pose.Y().value());
    frc::SmartDashboard::PutNumber("pose A", m_pose.Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("target pose X", m_targetPose.X().value());
    frc::SmartDashboard::PutNumber("target pose Y", m_targetPose.Y().value());
    frc::SmartDashboard::PutNumber("target pose A", m_targetPose.Rotation().Degrees().value());
}

void Swerve::driveTeleop() {
    int invert = 1;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        invert = -1;
    }
    Eigen::Vector2d velocity(-m_driverController.GetRawAxis(1) * invert, -m_driverController.GetRawAxis(0) * invert);
    double angularVelocity = -m_driverController.GetRawAxis(4);
    if (m_driverController.GetRawButton(4)) {
        resetRotation(0_deg);
    }
    // apply smooth deadband
    if (velocity.norm() > dB) {
        velocity *= (1.0F - dB/velocity.norm())/(1.0F - dB);
    } else { velocity = Eigen::Vector2d(0, 0); }
    if (abs(angularVelocity) > dB) {
        angularVelocity *= (1.0 - dB/abs(angularVelocity))/(1.0 - dB);
    } else { angularVelocity = 0; }
    velocity *= SwerveConstants::max_m_per_sec.value();
    angularVelocity *= SwerveConstants::max_rad_per_sec.value();
    frc::ChassisSpeeds rawControllerFieldOrientedSpeeds = frc::ChassisSpeeds{units::velocity::meters_per_second_t{velocity[0]},
                                                            units::velocity::meters_per_second_t{velocity[1]},
                                                            units::angular_velocity::radians_per_second_t{angularVelocity}};
    frc::ChassisSpeeds robotOrientedSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(rawControllerFieldOrientedSpeeds, m_pose.Rotation());
    auto states = m_kinematics.ToSwerveModuleStates(robotOrientedSpeeds);
    double desaturationValue = Util::desaturateChassisSpeeds(robotOrientedSpeeds, states);
    frc::ChassisSpeeds fieldRelativeSpeeds = rawControllerFieldOrientedSpeeds * desaturationValue;
    m_slewLimiter.Run(fieldRelativeSpeeds, SwerveConstants::time_to_full_speed, 0.02_s);
    states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(m_slewLimiter.GetSpeeds(), m_pose.Rotation()));
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredState(states[i]);
    }
}

frc2::CommandPtr Swerve::defaultDrive() {
    return Run([this] { driveTeleop(); }).WithName("Driving Teleoperated");
}

void Swerve::setTrajectory(const choreo::Trajectory<choreo::SwerveSample> & trajectory) {
    if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
        m_trajectory = trajectory.Flipped();
    } else {
        m_trajectory = trajectory;
    }
    autoTimer.Restart();
}

void Swerve::moveToNextSample() {
    if (!autoTimer.HasElapsed(m_trajectory.GetTotalTime())) {
        choreo::SwerveSample currentSample = m_trajectory.SampleAt(autoTimer.Get()).value();
        auto positionErrorX = currentSample.x - m_pose.X();
        auto positionErrorY = currentSample.y - m_pose.Y();
        auto headingError = currentSample.heading - m_pose.Rotation().Radians();
        am::limit(headingError);
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                positionErrorX*position_P/1_s + currentSample.vx,
                positionErrorY*position_P/1_s + currentSample.vy,
                headingError*heading_P/1_s + currentSample.omega,
                m_pose.Rotation());
        auto moduleStates = m_kinematics.ToSwerveModuleStates(speeds);
        m_kinematics.DesaturateWheelSpeeds(&moduleStates, max_m_per_sec);
        for (int i = 0; i < 4; i++) {
            m_moduleList[i]->setDesiredState(moduleStates[i]);
        }
    } else {
        for (auto &module : m_moduleList) {
            module->brake();
        }
    }
}

void Swerve::driveToTargetPose() {
    auto translationError = m_targetPose.Translation() - m_pose.Translation();
    auto rotationError = m_targetPose.Rotation() - m_pose.Rotation();
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            translationError.X()*position_P/1_s,
            translationError.Y()*position_P/1_s,
            rotationError.Radians()*heading_P/1_s,
            m_pose.Rotation());
    auto moduleStates = m_kinematics.ToSwerveModuleStates(speeds);
    m_kinematics.DesaturateWheelSpeeds(&moduleStates, max_limelight_m_per_sec);
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredState(moduleStates[i]);
    }
}

bool Swerve::targetPoseReached() {
    return m_targetPose.Translation().Distance(m_pose.Translation()) < 0.5_in;
}

bool Swerve::targetPoseReachedFor(units::second_t settleTime) {
    if (!targetPoseReached()) {
        positionReachedTimer.Restart();
    }
    return positionReachedTimer.HasElapsed(settleTime);
}

frc2::CommandPtr Swerve::setInitialTrajectoryCmd(const choreo::Trajectory<choreo::SwerveSample> & trajectory) {
    return RunOnce([this, trajectory] { setInitialTrajectory(trajectory); }).WithName("Setting initial trajectory"); 
}

frc2::CommandPtr Swerve::followTrajectory(const choreo::Trajectory<choreo::SwerveSample> & trajectory) {
    return frc2::FunctionalCommand(
        [this, trajectory] { setTrajectory(trajectory); },
        [this] { moveToNextSample(); },
        [this] (bool x) { brake(); },
        [this, trajectory] { return autoTimer.HasElapsed(trajectory.GetTotalTime()); },
        {this}
    ).ToPtr().WithName("Following Trajectory");
}

void Swerve::simpleDrive(frc::ChassisSpeeds robotOrientedSpeeds) {
    auto states = m_kinematics.ToSwerveModuleStates(robotOrientedSpeeds);
    m_kinematics.DesaturateWheelSpeeds(&states, max_m_per_sec);
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredState(states[i]);
    }
}

void Swerve::brake() {
    for (auto &module : m_moduleList) {
        module->brake();
    }
}

frc2::CommandPtr Swerve::driveToPole(const bool & isLeft) {
    return frc2::FunctionalCommand(
        [this, isLeft] { m_targetPose = getCoralScoringTargetPose(isLeft); },
        [this] { driveToTargetPose(); },
        [this] (bool x) { simpleDrive(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}); },
        [this] { return targetPoseReachedFor(0.5_s); },
        {this}
    ).ToPtr().WithName("driving to given reef pole");
}

frc2::CommandPtr Swerve::driveToPoleIntermediate(const bool & isLeft) {
    return frc2::FunctionalCommand(
        [this, isLeft] { m_targetPose = getIntermediateCoralScoringPose(isLeft); },
        [this] { driveToTargetPose(); },
        [this] (bool x) { simpleDrive(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}); },
        [this] { return targetPoseReached(); },
        {this}
    ).ToPtr().WithName("driving to intermediate coral scoring pose");
}

frc::Pose2d Swerve::getCoralScoringTargetPose(bool isLeft) {
    frc::Translation2d translationFromBlueReef = m_pose.Translation() - blueReefTranslation;
    frc::Translation2d translationFromRedReef = m_pose.Translation() - redReefTranslation;
    frc::Pose2d targetPose;
    if (translationFromBlueReef.Norm() < 2.5_m) {
        auto reefSideAngle = frc::Rotation2d{60_deg * int((translationFromBlueReef.Angle().Degrees().value() + 390)/60)};
        auto targetTranslation = blueReefTranslation + frc::Translation2d{units::meter_t{reefSideAngle.Cos()}, units::meter_t{reefSideAngle.Sin()}} * reefToRobotDistance;
        auto targetRotation = frc::Rotation2d{reefSideAngle} + frc::Rotation2d{180_deg};
        targetPose = frc::Pose2d{targetTranslation, targetRotation};
    }
    if (translationFromRedReef.Norm() < 2.5_m) {
        auto reefSideAngle = frc::Rotation2d{60_deg * int((translationFromRedReef.Angle().Degrees().value() + 390)/60)};
        auto targetTranslation = redReefTranslation + frc::Translation2d{units::meter_t{reefSideAngle.Cos()}, units::meter_t{reefSideAngle.Sin()}} * reefToRobotDistance;
        auto targetRotation = frc::Rotation2d{reefSideAngle} + frc::Rotation2d{180_deg};
        targetPose = frc::Pose2d{targetTranslation, targetRotation};
    }
    if (isLeft) {
        return {targetPose.Translation() + frc::Translation2d{-6.5_in*targetPose.Rotation().Sin(), 6.5_in*targetPose.Rotation().Cos()}, targetPose.Rotation()};
    } else {
        return {targetPose.Translation() + frc::Translation2d{6.5_in*targetPose.Rotation().Sin(), -6.5_in*targetPose.Rotation().Cos()}, targetPose.Rotation()};
    }
}

frc::Pose2d Swerve::getIntermediateCoralScoringPose(bool isLeft) {
    auto targetPose = getCoralScoringTargetPose(isLeft);
    return {targetPose.Translation()
        + frc::Translation2d{units::meter_t{-targetPose.Rotation().Cos()},
                             units::meter_t{-targetPose.Rotation().Sin()}} * scoringOffsetMeters,
                             targetPose.Rotation()};
}

bool Swerve::reefWithinRange() {
    frc::Translation2d translationFromBlueReef = m_pose.Translation() - blueReefTranslation;
    frc::Translation2d translationFromRedReef = m_pose.Translation() - redReefTranslation;
    return translationFromBlueReef.Norm() < 2.5_m || translationFromRedReef.Norm() < 2.5_m;
}

bool Swerve::safeToMoveCoralManipulator() {
    frc::Translation2d translationFromBlueReef = m_pose.Translation() - blueReefTranslation;
    frc::Translation2d translationFromRedReef = m_pose.Translation() - redReefTranslation;
    return (translationFromBlueReef.Norm() > units::meter_t{safeReefDistanceMeters})
        && (translationFromRedReef.Norm() > units::meter_t{safeReefDistanceMeters});
}

void Swerve::resetPosition(frc::Translation2d newTranslation) {
    m_pose = {newTranslation, m_pose.Rotation()};
}

void Swerve::resetRotation(frc::Rotation2d newRotation) {
    gyro.SetYaw(newRotation.Degrees());
    LimelightHelpers::SetRobotOrientation("", newRotation.Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
    m_pose = {m_pose.Translation(), newRotation};
}

void Swerve::resetPose(frc::Pose2d newPose) {
    resetPosition(newPose.Translation());
    resetRotation(newPose.Rotation());
    m_pose = newPose;
}

// sets the initial robot pose to be equal to the initial sample pose
void Swerve::setInitialTrajectory(const choreo::Trajectory<choreo::SwerveSample> & trajectory) {
    if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
        resetPose(trajectory.Flipped().GetInitialPose().value());
    } else {
        resetPose(trajectory.GetInitialPose().value());
    }
}

void Swerve::OdometryThread() {
    while (true) {
        BaseStatusSignal::WaitForAll(10_ms, m_statusSignals);
        m_pose = {m_pose.Translation(), m_gyroAngleSignal->GetValue()};
        LimelightHelpers::SetRobotOrientation("", m_pose.Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        frc::Translation2d deltaTranslationAverage{};
        for (auto & module : m_moduleList) {
            deltaTranslationAverage = deltaTranslationAverage + module->GetDeltaTranslation();
        }
        deltaTranslationAverage = deltaTranslationAverage * 0.25;
        deltaTranslationAverage.RotateBy(m_pose.Rotation());
        m_pose = m_pose + frc::Transform2d{deltaTranslationAverage, 0_deg};
        // Get the pose estimate
        LimelightHelpers::PoseEstimate limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");
        m_pastTranslations[m_currentTranslationIndex] = m_pose.Translation();
        if (m_validPastTranslationCount < 100) {
            m_validPastTranslationCount++;
        }
        if (limelightMeasurement.pose != m_lastLimelightPose && limelightMeasurement.tagCount != 0) {
            double latency = LimelightHelpers::getLatency_Capture() + LimelightHelpers::getLatency_Pipeline();
            int compensationCycles = latency*0.2;
            if (compensationCycles > m_validPastTranslationCount-1) {
                compensationCycles = m_validPastTranslationCount-1;
            }
            if (compensationCycles > 99) {
                compensationCycles = 99;
            }
            frc::Translation2d distanceSinceCapture = m_pastTranslations[m_currentTranslationIndex]
                                                    - m_pastTranslations[(m_currentTranslationIndex - compensationCycles + 100) % 100];
            m_pose = frc::Pose2d{limelightMeasurement.pose.Translation() + distanceSinceCapture, m_pose.Rotation()};
            m_validPastTranslationCount = 0;
            m_lastLimelightPose = limelightMeasurement.pose;
        }
        m_currentTranslationIndex++;
        if (m_currentTranslationIndex > 99) {
            m_currentTranslationIndex = 0;
        }
    }
}

void Swerve::InitializeOdometry() {
    for (auto & module : m_moduleList) {
        module->InitializeOdometry();
    }
    LimelightHelpers::SetIMUMode("", 0);
    std::thread odometryThread(&Swerve::OdometryThread, this);
    odometryThread.detach();
}

// set correct yaw depending on side of field
void Swerve::InitializeYaw() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value()) {
        if (alliance.value() == frc::DriverStation::Alliance::kBlue) {
            resetRotation(180_deg);
            cout << "initialized yaw for blue side\n";
        } else {
            resetRotation(0_deg);
            cout << "initialized yaw for red side\n";
        }
    }
}

Swerve::~Swerve() {
    delete m_gyroAngleSignal;
}