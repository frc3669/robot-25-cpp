#include "subsystems/Turret.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <thread>


Turret::Turret(Swerve * drivePtr, int turretControllerPortNum) : 
                 m_turretController(turretControllerPortNum) {    
    // add all the status signals to a list for syncronized updates
    m_drivePtr = drivePtr;   // Get access to the Swerve susbsystem, so the pose can be returned
}  
 
void Turret::SimulationPeriodic() {}

void Turret::Periodic() {
 
     // ****************************************
    string selectedShooterTarget = m_shooterTgtChooser.GetSelected();
    if (selectedShooterTarget == "BLUEHub") {
        setTurretTarget (m_BLUE_TargetHub);
    }
    else if (selectedShooterTarget == "REDHub") {
        setTurretTarget (m_RED_TargetHub);
    }
    // ****************************************

    // ****************************************
    // Determine Turret Angle and Distance to the Target
    // (Since Turret Target has been Identified) 
    if (m_turretTargetSet)
    {
        // Get the current robot pose
        frc::Pose2d  m_pose = m_drivePtr->getPose();

        // Determine the Turret Pose (Relative to the Robot Pose)
         m_turretPose = m_pose + m_relativeTurretPose;

        // Compute Shooting solution (stationary) Turret Angle and Distance
        // The turret angle will point to the target, compensating for robot heading.
        m_turretTargetAngle    = computeTurretAngleInDegrees(m_turretPose,
                                                             m_turretTarget );
        m_turretTargetDistance = computeDistanceInMeters(m_turretPose.X().value(),
                                                         m_turretPose.Y().value(),
                                                         m_turretTarget.X().value(),
                                                         m_turretTarget.Y().value());
    }
    // ****************************************       

    // ****************************************
    frc::SmartDashboard::PutString("TurretTargetID", selectedShooterTarget);  
    frc::SmartDashboard::PutNumber("TurretTargetX", m_turretTarget.X().value());  
    frc::SmartDashboard::PutNumber("TurretTargetY", m_turretTarget.Y().value());  
 
    frc::SmartDashboard::PutNumber("TurretX", m_turretPose.X().value());  
    frc::SmartDashboard::PutNumber("TurretY", m_turretPose.Y().value());    
    frc::SmartDashboard::PutNumber("TurretAngle", m_turretTargetAngle);  
    frc::SmartDashboard::PutNumber("TurretDistance", m_turretTargetDistance);  
    // ****************************************
}



// ****************************************
void Turret::setTurretTarget (frc::Translation2d theShooterTarget) {
    // Set the Target for the Turret
    m_turretTarget = {(units::length::meter_t) theShooterTarget.X().value(),
                      (units::length::meter_t) theShooterTarget.Y().value()};
    m_turretTargetSet = true;
}

frc::Translation2d Turret::getTurretTarget () {
    // Set the Target for the Turret
    return (m_turretTarget);
}
// ****************************************

void Turret::pointTurretAtTarget () {
    // Turn the Turret to point at the Target
}
// ****************************************


// ****************************************
double Turret::computeDistanceInMeters(double x1, double y1, double x2, double y2) {
    double x_diff = x2 - x1;
    double y_diff = y2 - y1;
    // Apply the distance formula
    double distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    return distance;
}

// NOTE: robotPose is ASSUMED to be the center of the robot.
//       The robotPose must be converted to turretPose.
//       It is ASSUMED that robotPose has ALREADY accounted for camera(s) placement(s) 
//       withinn the odometry processing.
//    
//       The turretPose must use robotPose and account for turret placement.
//       (Turret Offset from robot center AND rotation on the field due to robot heading.)
//
double Turret::computeTurretAngleInDegrees(frc::Pose2d robotPose, frc::Translation2d turretTarget )
{
    double pi_val = 4.0 * std::atan(1.0);       // Compute PI to many digits, atan (1 radian) = PI/4
    double RadiansToDegrees = 180.0 / pi_val;   // Radians to Degrees Conversion Factor
    //double DegreesToRadians = pi_val / 180.0; // Degrees to Radians Conversion Factor
    double TurretAngleDegrees = 0.0;
    double MAX_TURRET_ROTATION_ANGLE = 270.0;   // Maximum Turret Rotation Angle (Either Direction)
    //double TurretAngleRadians = 0.0;

    // Compute the Angle from the Robot X,Y Position to the Target X,Y Position
    double delta_X = (double) (turretTarget.X() - robotPose.X());
    double delta_Y = (double) (turretTarget.Y() - robotPose.Y());
    double robotToTgtAngleRadians = atan2(delta_Y, delta_X);                    // Radians
    double robotToTgtAngleDegrees = robotToTgtAngleRadians * RadiansToDegrees;  // Degrees

    // Turret Angle to Target -  based on Robot Heading
    TurretAngleDegrees = robotToTgtAngleDegrees - (double) (robotPose.Rotation().Degrees());
 
    // Keep Turret Angle within physical limits
    if (TurretAngleDegrees <= -(MAX_TURRET_ROTATION_ANGLE)) {
        TurretAngleDegrees += 360.0;
    } else if (TurretAngleDegrees >= MAX_TURRET_ROTATION_ANGLE) {
        TurretAngleDegrees -= 360.0;
    }

    //TurretAngleRadians = TurretAngleDegrees *  DegreesToRadians;

    return (TurretAngleDegrees);
}
// ****************************************

// Turret Destructor
Turret::~Turret() {}
