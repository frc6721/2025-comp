// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;//4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of  the modules relative to thechassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0.0 ;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0.0 ;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontLeftTurningCanId = 2;

    
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kFrontRightTurningCanId = 4;
    

    public static final int kRearLeftDrivingCanId = 5;
    public static final int kRearLeftTurningCanId = 6;

    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 6.75;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
    public static final double kDriveDeadband = 0.05;
    
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ClimberConstants {
    public static final int kClimberID = 10;
    public static final int kPhotoID = 1;

    public static final double kClimbSpeed = .5;

    public static final double kClimbPos = 190;
    public static final double kStow = 0.0;

       /***********************
     * PID
     ***********************/
    public static final double kClimberkP = 10.25;
    public static final double kClimberkI = 0;
    public static final double kClimberkD = 0.1;
    public static final double kMaxVel = 5000;
    public static final double kMaxAccel = 6000;
    public static final TrapezoidProfile.Constraints Constraints = new 
    TrapezoidProfile.Constraints(kMaxVel,kMaxAccel);
  }

  public static final class ElevatorConstants {
    public static final int kElevator1ID = 11;
    public static final int kElevator2ID = 12;

    public static final double kStow = 0.0;
    public static final double kL1 = 5.0;
    public static final double kL2 = 13.0;
    public static final double kL3 = 29.0;
    public static final double kL4 = 57.0;

    //Elevator gear ratio is 12:1
    public static final double kElevTestSpeed = 0.1;

      /***********************
     * PID
     ***********************/
    public static final double kElevatorkP = 0.35;
    public static final double kElevatorkI = 0;
    public static final double kElevatorkD = 0.0;
    public static final double kElevatorkFF = 0;
    public static final double kMaxVel = 5000;
    public static final double kMaxAccel = 6000;
    public static final TrapezoidProfile.Constraints Constraints = new 
    TrapezoidProfile.Constraints(kMaxVel,kMaxAccel);
    
    

    
    

  }

  public static final class ActuatorConstants {
    public static final int kIntakeID = 13;
    public static final int kBreakBeam1ID = 0;

    public static final double kActuatorSpeed = 0.60;
    public static final double kAutoActuatorSpeed = 0.35;

  }
}
