// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;//was 5.8462, changed to mk4i L2 6.75
        public static final double kTurningMotorGearRatio = 1 / 21.426;//was 18, changed to mk4i 150/7:1 or 21.426
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
  }

   public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(18.63);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        
        //Switched lefts and rights since it was done as if it was upside down.
        public static final int kFrontLeftDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackRightDriveMotorPort = 15;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 16;
        public static final int kFrontRightTurningMotorPort = 13;
        public static final int kBackRightTurningMotorPort = 14;

        public static final int kPigeonPort = 5;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //need to double check these
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;//0
        public static final int kBackLeftDriveAbsoluteEncoderPort = 16;//2
        public static final int kFrontRightDriveAbsoluteEncoderPort = 13;//1
        public static final int kBackRightDriveAbsoluteEncoderPort = 14;//3

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;//(-2.112 * 2);// / 4096) * 2 * Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;//(3.608 * 2);// / 4096) * 2 * Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;// (4.016 * 2);//4096) * 2 * Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;//(0.800 * 2);// 4096) * 2 * Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
      public static final int kDriverControllerPort = 0;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.05;
    }
}
