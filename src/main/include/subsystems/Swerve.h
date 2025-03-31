#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/GenericHID.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "subsystems/SwerveModule.h"
#include <frc/Timer.h>
#include <complex.h>
#include "choreo/Choreo.h"
#include "Constants.h"

class Swerve : public frc2::SubsystemBase {
  public:
    Swerve(frc::GenericHID *driverController);
    void Periodic() override;
    void SimulationPeriodic() override;
    frc2::CommandPtr defaultDrive();
    frc2::CommandPtr followTrajectory(choreo::Trajectory<choreo::SwerveSample> &trajectory);
    void brake();
    frc2::CommandPtr driveRightToPole();
    frc2::CommandPtr driveLeftToPole();
    frc2::CommandPtr resetPoseCmd(complex<float> position, float angle);
    frc2::CommandPtr resetPositionCmd(complex<float> position);
    void addModule(SwerveModule &module);
    
  private:
    frc::GenericHID *driverController;
    ctre::phoenix6::hardware::Pigeon2 gyro{1, "CTREdevices"};
    frc::DigitalInput poleSensor{1};
    frc::Timer autoTimer;
    std::vector<SwerveModule> modules;
    complex<float> slewVelocity = complex<float>(0, 0);
    float slewAngularVelocity = 0;
    complex<float> position = complex<float>(0, 0);
    float startingAngle = 0;
    float heading = 0;
    choreo::Trajectory<choreo::SwerveSample> *trajectory;
    float possibleReefAngles[6] = {0, M_PI/3, 2*M_PI/3, M_PI, 4*M_PI/3, 5*M_PI/3};
    float possibleFeederStationAngles[2] = {2.1995556168958954, -2.1995556168958954};
    
    void setTrajectory(choreo::Trajectory<choreo::SwerveSample> *trajectory);
    void moveToNextSample();
    float getReefAlignmentError();
    float getFeederStationAlignmentError();
    void calculateOdometry();
    void resetPosition(complex<float> position);
    void resetPose(complex<float> position, float angle);
    void driveTeleop();
    void simpleDrive(complex<float> velocity);
};