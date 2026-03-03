#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/SwerveModule.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include "Constants.h"
#include "util.h"

#include "Swerve.h"

class Turret : public frc2::SubsystemBase {
  public:
    Turret(Swerve * drivePtr, int turretControllerPortNum);

    void Periodic() override;
    void SimulationPeriodic() override;

    ~Turret();

    // Pointer for Access to the Swerve Subsystem
    Swerve * m_drivePtr;

    // ****************************************
    // Target Shooter Chooser
    frc::SendableChooser<string> m_shooterTgtChooser;


    // Indicates whether the Turret Target Pose has been SET
    bool m_turretTargetSet;
    // Turret Angle to the currently selected target 
    double m_turretTargetAngle;
    // Turret Distance to the currently selected target
    double m_turretTargetDistance;
    // 
    // The currently selected turret target position
    frc::Translation2d m_turretTarget {0_in, 0_in};

     // Pre-defined TurretTargetPose Selections  (X, Y) in INCHES
    frc::Translation2d m_BLUE_TargetHub{182.11_in, 158.845_in};
    frc::Translation2d m_RED_TargetHub {469.11_in, 158.845_in};   

    // Set/Get Turret Target Location (X,Y)
    void setTurretTarget (frc::Translation2d theShooterTarget); 
    frc::Translation2d getTurretTarget ();   
    // ****************************************


  private:
    frc::GenericHID m_turretController;
   
    // target pose for autonomous positioning during teleop
    frc::Pose2d m_targetPose;
    double lastTurretAngle = 0;

    // ****************************************
    // Turret Pose (Determined from Robot Pose and turret placement offsets)
    // NOTE: Robot Pose is Front Facing, with position at the Center of the Robot
    frc::Pose2d m_turretPose;

    // Relative Turret Offset (x,y in meters) from Robot Center, Pointing the SAME DIRECTION as the Robot.
    // (Since facing the same direction as the robot, angle is ZERO).
    // (Translates inches to meters accomplished by the type definitions)
    // (TBD - X= -12.0 inches, Y = +12.0 inches, NO ROTATION !!!)
    frc::Translation2d m_turretTranslation{ units::length::meter_t {-12.0_in},
                                            units::length::meter_t {+12.0_in}};

                    
    // ****************************************

   
    double computeDistanceInMeters(double x1, double y1, double x2, double y2);

    double computeTurretAngleInDegrees(frc::Pose2d robotPose, frc::Translation2d turretTargetPose );

    // drive to the currently set target pose
    void pointTurretAtTarget();
    
};