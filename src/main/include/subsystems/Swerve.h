#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/GenericHID.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include "subsystems/SwerveModule.h"
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <complex.h>
#include "choreo/Choreo.h"
#include "Constants.h"
#include "util.h"
#include <LimelightHelpers.h>

class Swerve : public frc2::SubsystemBase {
  public:
    Swerve(int driverControllerPortNum);
    void Periodic() override;
    void SimulationPeriodic() override;
    frc2::CommandPtr defaultDrive();
    frc2::CommandPtr followTrajectory(const choreo::Trajectory<choreo::SwerveSample> & trajectory);
    void brake();
    frc2::CommandPtr driveToRightPole();
    frc2::CommandPtr driveToLeftPole();
    frc2::CommandPtr setInitialTrajectoryCmd(const choreo::Trajectory<choreo::SwerveSample> & trajectory);
    void InitializeOdometry();
    void InitializeYaw();
    bool reefWithinRange();
    ~Swerve();
    
  private:
    frc::GenericHID m_driverController;
    ctre::phoenix6::hardware::Pigeon2 gyro{1, "CTREdevices"};
    ctre::phoenix6::StatusSignal<units::angle::degree_t> * m_gyroAngleSignal;
    std::vector<ctre::phoenix6::BaseStatusSignal*> m_statusSignals;
    frc::Timer autoTimer;
    // robot swerve modules
    SwerveModule m_frontLeft = SwerveModule(1), m_backLeft = SwerveModule(2),
        m_backRight = SwerveModule(3), m_frontRight = SwerveModule(4);
    SwerveModule * m_moduleList[4] = {&m_frontLeft, &m_backLeft,
                                    &m_backRight, &m_frontRight};
    frc::Translation2d m_frontLeftLocation{12_in, 12_in};
    frc::Translation2d m_backLeftLocation{-12_in, 12_in};
    frc::Translation2d m_backRightLocation{-12_in, -12_in};
    frc::Translation2d m_frontRightLocation{12_in, -12_in};
    frc::SwerveDriveKinematics<4> m_kinematics {
      m_frontLeftLocation, m_backLeftLocation,
      m_backRightLocation, m_frontRightLocation
    };
    frc::ChassisSpeeds m_rawControllerFieldOrientedSpeeds;
    Util::SlewLimiter m_slewLimiter;
    complex<units::velocity::meters_per_second_t> slewVelocity = complex<units::velocity::meters_per_second_t> (0_mps, 0_mps);
    units::angular_velocity::radians_per_second_t slewAngularVelocity = 0_rad_per_s;
    units::velocity::meters_per_second_t m_xVelocity, m_yVelocity;
    units::angular_velocity::radians_per_second_t m_angularRate;
    frc::Pose2d m_pose;
    frc::Pose2d m_lastLimelightPose;
    frc::Pose2d m_targetPose;
    frc::Translation2d m_pastTranslations[100];
    int m_validPastTranslationCount = 0;
    int m_currentTranslationIndex = 0;
    choreo::Trajectory<choreo::SwerveSample> m_trajectory;

    void setInitialTrajectory(const choreo::Trajectory<choreo::SwerveSample> & trajectory);
    void setTrajectory(const choreo::Trajectory<choreo::SwerveSample> & trajectory);
    void moveToNextSample();
    void driveToTargetPose();
    bool targetPoseReached();
    void resetPosition(frc::Translation2d newTranslation);
    void resetRotation(frc::Rotation2d newRotation);
    void resetPose(frc::Pose2d newPose);
    void driveTeleop();
    void simpleDrive(frc::ChassisSpeeds robotOrientedSpeeds);
    void setCoralScoringTargetPose(bool isLeft);
    void OdometryThread();
};