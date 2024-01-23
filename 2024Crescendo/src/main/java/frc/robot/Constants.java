// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Swerve {
    
    public static final double trackWidth = 0.6254; // Units.inchesToMeters(24.625);
    public static final double wheelBase = 0.6096; // units.inchesToMeters(24.625)
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (21.42857 / 1.0); // 12.8:1

    /* Angle Motor PID Values */
    /*
     * KP = Propotional Gain constant of the PIDF controller
     * KI = Integral Gain constant of the PIDF controller
     * KD = Derivitive Gain constant of the PIDF controller
     * KFF = Feed-forward gain of the PIDF controller
     * Use SPARKMAX GUI to tune and save paremeters
     */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Charcterization Vals */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    
    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;
    
    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // 4.5 meters per second
    public static final double maxAngularVelocity = 11.5;// 11.5
    // var speeds = new chassisspeeds(3.0,-2.0, Math.pi);
    // TODO turtle mode
    public static final double turtleSpeed = 1.35; // 1.35 meters per second

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final class FrontRightSwerveModule {
      public static final int AngleMotorID = 11;
      public static final int DriveMotorID = 12;
      public static final int CANcoderID = 13;
      public static final Rotation2d AngleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants Constants = new SwerveModuleConstants();
      public FrontRightSwerveModule() {
        Constants.CANcoderId = CANcoderID;
        Constants.DriveMotorId = DriveMotorID;
        Constants.SteerMotorId = AngleMotorID;
        Constants.CANcoderOffset = AngleOffset.getDegrees();
      }
    }
    public static final class FrontLeftSwerveModule {
      public static final int AngleMotorID = 21;
      public static final int DriveMotorID = 22;
      public static final int CANcoderID = 23;
      public static final Rotation2d AngleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants Constants = new SwerveModuleConstants();
      public FrontLeftSwerveModule() {
        Constants.CANcoderId = CANcoderID;
        Constants.DriveMotorId = DriveMotorID;
        Constants.SteerMotorId = AngleMotorID;
        Constants.CANcoderOffset = AngleOffset.getDegrees();
      }
    }
    public static final class BackRightSwerveModule {
      public static final int AngleMotorID = 14;
      public static final int DriveMotorID = 15;
      public static final int CANcoderID = 16;
      public static final Rotation2d AngleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants Constants = new SwerveModuleConstants();
      public BackRightSwerveModule() {
        Constants.CANcoderId = CANcoderID;
        Constants.DriveMotorId = DriveMotorID;
        Constants.SteerMotorId = AngleMotorID;
        Constants.CANcoderOffset = AngleOffset.getDegrees();
      }
    }
    public static final class BackLeftSwerveModule {
      public static final int AngleMotorID = 24;
      public static final int DriveMotorID = 25;
      public static final int CANcoderID = 26;
      public static final Rotation2d AngleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants Constants = new SwerveModuleConstants();
      public BackLeftSwerveModule() {
        Constants.CANcoderId = CANcoderID;
        Constants.DriveMotorId = DriveMotorID;
        Constants.SteerMotorId = AngleMotorID;
        Constants.CANcoderOffset = AngleOffset.getDegrees();
      }
    }
  }
}
