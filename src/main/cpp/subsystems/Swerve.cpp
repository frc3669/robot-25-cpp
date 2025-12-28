#include "subsystems/Swerve.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <thread>

using namespace SwerveConstants;
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

void Swerve::Periodic() {}

void Swerve::driveTeleop() {
    complex<double> velocity = complex<double>(-m_driverController.GetRawAxis(1), -m_driverController.GetRawAxis(0));
    double angularVelocity = -m_driverController.GetRawAxis(4);
    if (m_driverController.GetRawButton(4)) {
        resetRotation(0_deg);
    }
    // apply smooth deadband
    if (abs(velocity) > dB) {
        velocity *= (1.0F - dB/abs(velocity))/(1.0F - dB);
    } else { velocity = complex<double>(0, 0); }
    if (abs(angularVelocity) > dB) {
        angularVelocity *= (1.0 - dB/abs(angularVelocity))/(1.0 - dB);
    } else { angularVelocity = 0; }
    // autoalign with A and B buttons
    if (m_driverController.GetRawButton(1)) {
        angularVelocity += getReefAlignmentError().value() * autoalign_P;
    } else if (m_driverController.GetRawButton(2)) {
        angularVelocity += getFeederStationAlignmentError().value() * autoalign_P;
    }
    velocity *= SwerveConstants::max_m_per_sec.value();
    angularVelocity *= SwerveConstants::max_rad_per_sec.value();
    m_rawControllerFieldOrientedSpeeds = frc::ChassisSpeeds{units::velocity::meters_per_second_t{velocity.real()},
                                                            units::velocity::meters_per_second_t{velocity.imag()},
                                                            units::angular_velocity::radians_per_second_t{angularVelocity}};
    frc::ChassisSpeeds robotOrientedSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(m_rawControllerFieldOrientedSpeeds, m_pose.Rotation());
    auto states = m_kinematics.ToSwerveModuleStates(robotOrientedSpeeds);
    double desaturationValue = Util::desaturateChassisSpeeds(robotOrientedSpeeds, states);
    frc::ChassisSpeeds fieldRelativeSpeeds = m_rawControllerFieldOrientedSpeeds * desaturationValue;
    m_slewLimiter.Run(fieldRelativeSpeeds, SwerveConstants::time_to_full_speed, 0.02_s);
    states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(m_slewLimiter.GetSpeeds(), m_pose.Rotation()));
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredStateTeleop(states[i]);
    }
}

frc2::CommandPtr Swerve::defaultDrive() {
    return Run([this] { driveTeleop(); }).WithName("Driving Teleoperated");
}

void Swerve::moveToNextSample(choreo::Trajectory<choreo::SwerveSample> *trajectory) {   
    if (!autoTimer.HasElapsed(trajectory->GetTotalTime())) {
        choreo::SwerveSample currentSample = trajectory->SampleAt(autoTimer.Get()).value();
        auto positionErrorX = currentSample.x - m_pose.X();
        auto positionErrorY = currentSample.y - m_pose.Y();
        auto headingError = currentSample.heading - m_pose.Rotation().Radians();
        am::limit(headingError);
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                positionErrorX*position_P/1_s + currentSample.vx,
                positionErrorY*position_P/1_s + currentSample.vy,
                headingError*heading_P/1_s + currentSample.omega,
                m_pose.Rotation());
        // frc::ChassisSpeeds accelerations = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        //         currentSample.ax.value()*1_mps_sq,
        //         currentSample.ay.value()*1_mps_sq,
        //         currentSample.alpha.value()*1_rad_per_s_sq,
        //         m_odometry.GetPose().Rotation());
        auto moduleStates = m_kinematics.ToSwerveModuleStates(speeds);
        m_kinematics.DesaturateWheelSpeeds(&moduleStates, max_m_per_sec);
        // auto moduleAccelerationStates = m_kinematics.ToSwerveModuleStates(accelerations);
        for (int i = 0; i < 4; i++) {
            m_moduleList[i]->setDesiredStateTeleop(moduleStates[i]);
        }
    } else {
        for (auto &module : m_moduleList) {
            module->brake();
        }
    }
}

frc2::CommandPtr Swerve::followTrajectory(choreo::Trajectory<choreo::SwerveSample> *trajectory) {
    return frc2::FunctionalCommand(
        [this] { this->autoTimer.Restart(); },
        [this, trajectory] { moveToNextSample(trajectory); },
        [this] (bool x) { brake(); },
        [this, trajectory] { return autoTimer.HasElapsed(trajectory->GetTotalTime()); },
        {this}
    ).ToPtr().WithName("Following Trajectory");
}

void Swerve::simpleDrive(frc::ChassisSpeeds robotOrientedSpeeds) {
    auto states = m_kinematics.ToSwerveModuleStates(robotOrientedSpeeds);
    m_kinematics.DesaturateWheelSpeeds(&states, max_m_per_sec);
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredStateTeleop(states[i]);
    }
}

void Swerve::brake() {
    for (auto &module : m_moduleList) {
        module->brake();
    }
}

frc2::CommandPtr Swerve::driveRightToPole() {
    return frc2::FunctionalCommand(
        [this] { simpleDrive(frc::ChassisSpeeds{0_mps, -0.5_mps, 0_rad_per_s}); },
        [this] { simpleDrive(frc::ChassisSpeeds{0_mps, -0.5_mps, 0_rad_per_s}); },
        [this] (bool x) { simpleDrive(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}); },
        [this] { return !poleSensor.Get(); },
        {this}
    ).ToPtr().WithName("Driving to Right Pole");
}

frc2::CommandPtr Swerve::driveLeftToPole() {
    return frc2::FunctionalCommand(
        [this] { simpleDrive(frc::ChassisSpeeds{0_mps, 0.5_mps, 0_rad_per_s}); },
        [this] { simpleDrive(frc::ChassisSpeeds{0_mps, 0.5_mps, 0_rad_per_s}); },
        [this] (bool x) { simpleDrive(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}); },
        [this] { return !poleSensor.Get(); },
        {this}
    ).ToPtr().WithName("Driving to Left Pole");
}

units::angle::radian_t Swerve::getReefAlignmentError() {
    for (auto &angle : possibleReefAngles) {
        auto error = angle - m_pose.Rotation().Radians();
        am::limit(error);
        if (units::math::abs(error) <= 1_rad*M_PI/6) {
            return error;
        }
    }
    return 0_rad;
}

units::angle::radian_t Swerve::getFeederStationAlignmentError() {
    auto error1 = possibleFeederStationAngles[0] - m_pose.Rotation().Radians();
    am::limit(error1);
    auto error2 = possibleFeederStationAngles[1] - m_pose.Rotation().Radians();
    am::limit(error2);
    if (units::math::abs(error1) < units::math::abs(error2)) {
        return error1;
    }
    return error2;
}

void Swerve::resetPosition(frc::Translation2d newTranslation) {
    m_pose = {newTranslation, m_pose.Rotation()};
}

void Swerve::resetRotation(frc::Rotation2d newRotation) {
    m_gyroOffset = m_gyroAngle - newRotation.Degrees();
    m_pose = {m_pose.Translation(), newRotation};
    
}

void Swerve::resetPose(frc::Pose2d newPose) {
    resetPosition(newPose.Translation());
    resetRotation(newPose.Rotation());
}

frc2::CommandPtr Swerve::resetPositionCmd(frc::Translation2d newTranslation) {
    return RunOnce([this, newTranslation] { resetPosition(newTranslation); }).WithName("Resetting Position to Specified Value");
}

frc2::CommandPtr Swerve::resetPoseCmd(frc::Pose2d newPose) {
    return RunOnce([this, newPose] { resetPose(newPose); }).WithName("Resetting Pose to Specified Value");
}

void Swerve::OdometryThread() {
    while (true) {
        BaseStatusSignal::WaitForAll(10_ms, m_statusSignals);
        m_gyroAngle = m_gyroAngleSignal->GetValue();
        m_pose = {m_pose.Translation(), m_gyroAngle - m_gyroOffset};
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
    resetRotation(0_deg);
    LimelightHelpers::SetRobotOrientation("", m_pose.Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers::SetIMUMode("", 3);
    std::thread odometryThread(&Swerve::OdometryThread, this);
    odometryThread.detach();
}

Swerve::~Swerve() {
    delete m_gyroAngleSignal;
}