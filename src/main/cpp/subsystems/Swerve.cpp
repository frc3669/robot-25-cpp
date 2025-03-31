#include "subsystems/Swerve.h"
#include <frc2/command/Commands.h>

using namespace SwerveConstants;
using namespace DriverControllerConstants;

Swerve::Swerve(frc::Joystick *driverController) {
    this->driverController = driverController;
}

void Swerve::SimulationPeriodic() {}

void Swerve::Periodic() {}

void Swerve::driveTeleop() {
    complex<float> velocity = (0.3F + float(driverController->GetRawAxis(3))*0.7F)*complex<float>(-driverController->GetRawAxis(1), -driverController->GetRawAxis(0));
    float angularVelocity = -driverController->GetRawAxis(4);
    if (driverController->GetRawButton(4)) {
        gyro.SetYaw(0_deg);
    }
    // apply smooth deadband
    if (abs(velocity) > dB) {
        velocity *= (1.0F - dB/abs(velocity))/(1.0F - dB);
    } else { velocity = complex<float>(0, 0); }
    if (abs(angularVelocity) > dB) {
        angularVelocity *= (1.0F - dB/abs(angularVelocity))/(1.0F - dB);
    } else { angularVelocity = 0; }
    // autoalign with A and B buttons
    heading = gyro.GetYaw().GetValueAsDouble()*M_PI/180 + startingAngle;
    // get complex number for robot orienting and field orienting
    complex<float> robotOrientMultiplier = polar<float>(1, -heading);
    if (driverController->GetRawButton(1)) {
        angularVelocity += getReefAlignmentError() * autoalign_P;
    } else if (driverController->GetRawButton(2)) {
        angularVelocity += getFeederStationAlignmentError() * autoalign_P;
    }
    // scale the velocities to meters per second
    velocity *= max_m_per_sec;
    angularVelocity *= max_m_per_sec;
    // find the robot oriented velocity
    complex<float> robotVelocity = velocity * robotOrientMultiplier;
    // find the fastest module speed
    float highest = max_m_per_sec;
    for (auto &module : modules) {
        float moduleSpeed = abs(module.findModuleVector(robotVelocity, angularVelocity));
        if (moduleSpeed > highest) { highest = moduleSpeed; }
    }
    // normalize the velocities
    velocity *= max_m_per_sec/highest;
    angularVelocity *= max_m_per_sec/highest;
    robotVelocity *= max_m_per_sec/highest;
    // find error between command and slew velocities
    complex<float> velocityError = velocity - slewVelocity;
    float angularVelocityError = angularVelocity - slewAngularVelocity;
    // find the robot oriented velocity error
    complex<float> robotVelocityError = velocityError * robotOrientMultiplier;
    complex<float> robotSlewVelocity = slewVelocity * robotOrientMultiplier;
    // find the max acceleration overshoot
    highest = 1;
    for (auto &module : modules) {
        float moduleOvershoot = module.getAccelOvershoot(robotSlewVelocity, slewAngularVelocity, robotVelocityError, angularVelocityError);
        if (moduleOvershoot > highest) { highest = moduleOvershoot; }
    }
    // normalize to find velocity increment
    complex<float> velocityIncrement = velocityError/highest;
    float angularVelocityIncrement = angularVelocityError/highest;
    // increment velocity
    if (abs(velocityError) > max_m_per_sec_per_cycle) {
        slewVelocity += velocityIncrement;
    } else {
        slewVelocity = velocity;
    }
    if (abs(angularVelocityError) > max_m_per_sec_per_cycle) {
        slewAngularVelocity += angularVelocityIncrement;
    } else {
        slewAngularVelocity = angularVelocity;
    }
    // update the robot oriented slew velocity
    robotSlewVelocity = slewVelocity * robotOrientMultiplier;
    // find acceleration feedforward
    complex<float> robotAccel = robotVelocityError*2.0F;
    float angularAccel = angularVelocityError*2.0F;
    // drive the modules
    for (auto &module : modules) {
        module.setVelocity(robotSlewVelocity, slewAngularVelocity, robotAccel, angularAccel);
    }
}

frc2::CommandPtr Swerve::defaultDrive() {
    return Run([this] { driveTeleop(); }).WithName("Driving Teleoperated");
}

void Swerve::setTrajectory(choreo::Trajectory<choreo::SwerveSample> *trajectory) {
    this->trajectory = trajectory;
    autoTimer.Restart();
}

void Swerve::moveToNextSample() {   
    heading = gyro.GetYaw().GetValueAsDouble()*M_PI/180 + startingAngle;
    if (!autoTimer.HasElapsed(trajectory->GetTotalTime())) {
        calculateOdometry();
        choreo::SwerveSample currentSample = trajectory->SampleAt(autoTimer.Get()).value();
        complex<float> positionError = complex<float>(currentSample.x.value(), currentSample.y.value());
        float headingError = currentSample.heading.value() - heading;
        am::limit(headingError);
        complex<float> velocity = complex<float>(currentSample.vx.value(), currentSample.vy.value()) = position_P * positionError;
        float angularVelocity = currentSample.omega.value() + heading_P * headingError;
        velocity *= polar<float>(1, -heading);
        for (auto &module : modules) {
            module.setVelocity(velocity, angularVelocity, complex<float>(currentSample.ax.value(), currentSample.ay.value()), currentSample.alpha.value());
        }
    } else {
        for (auto &module : modules) {
            module.setVelocity({0,0}, 0, {0,0}, 0);
        }
    }
}

frc2::CommandPtr Swerve::followTrajectory(choreo::Trajectory<choreo::SwerveSample> &trajectory) {
    return frc2::FunctionalCommand(
        [this, &trajectory] { setTrajectory(&trajectory); },
        [this] { moveToNextSample(); },
        [this] (bool x) { brake(); },
        [this, &trajectory] { return autoTimer.HasElapsed(trajectory.GetTotalTime()); },
        {this}
    ).ToPtr().WithName("Following Trajectory");
}

void Swerve::simpleDrive(complex<float> velocity) {
    float moduleSpeed = abs(velocity);
    if (moduleSpeed > max_m_per_sec) {
        velocity *= max_m_per_sec/moduleSpeed;
    }
    for (auto &module : modules) {
        module.setVelocity(velocity, 0, {0, 0}, 0);
    }
}

void Swerve::brake() {
    for (auto& module : modules) {
        module.brake();
    }
}

frc2::CommandPtr Swerve::driveRightToPole() {
    return frc2::FunctionalCommand(
        [this] { simpleDrive(complex<float>(0, -0.3)); },
        [this] { simpleDrive(complex<float>(0, -0.3)); },
        [this] (bool x) { simpleDrive(complex<float>(0, 0)); },
        [this] { return !poleSensor.Get(); },
        {this}
    ).ToPtr().WithName("Driving to Right Pole");
}

frc2::CommandPtr Swerve::driveLeftToPole() {
    return frc2::FunctionalCommand(
        [this] { simpleDrive(complex<float>(0, 0.3)); },
        [this] { simpleDrive(complex<float>(0, 0.3)); },
        [this] (bool x) { simpleDrive(complex<float>(0, 0)); },
        [this] { return !poleSensor.Get(); },
        {this}
    ).ToPtr().WithName("Driving to Left Pole");
}

float Swerve::getReefAlignmentError() {
    for (auto &angle : possibleReefAngles) {
        float error = angle - heading;
        am::limit(error);
        if (abs(error) <= M_PI/6) {
            return error;
        }
    }
    return 0;
}

float Swerve::getFeederStationAlignmentError() {
    float error1 = possibleFeederStationAngles[0] - heading;
    am::limit(error1);
    float error2 = possibleFeederStationAngles[1] - heading;
    am::limit(error2);
    if (abs(error1) < abs(error2)) {
        return error1;
    }
    return error2;
}

void Swerve::resetPosition(complex<float> position) {
    for (auto &module : modules) {
        module.resetEncoders();
    }
    this->position = position;
}

void Swerve::resetPose(complex<float> position, float angle) {
    resetPosition(position);
    startingAngle = angle;
}

frc2::CommandPtr Swerve::resetPositionCmd(complex<float> position) {
    return RunOnce([this, &position] { resetPosition(position); }).WithName("Resetting Position to Specified Value");
}

frc2::CommandPtr Swerve::resetPoseCmd(complex<float> position, float angle) {
    return RunOnce([this, &position, &angle] { resetPose(position, angle); }).WithName("Resetting Pose to Specified Value");
}

void Swerve::calculateOdometry() {
    complex<float> positionChange = complex<float>(0, 0);
    for (auto &module : modules) {
        positionChange += module.getPositionChange();
    }
    position += positionChange * polar<float>(0.25, heading);
}

void Swerve::addModule(SwerveModule &module) {
    modules.push_back(module);
}