// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    // These IDs are actually backwards because during rotation, the wheels would be an x
    public static final class CANDevices {
        public static final int powerDistributionHubId = 1;

        //Pigeon2 ID
        public static final int imuId = 2;

        public static final int frontLeftSteerMtrId = 11;
        public static final int frontLeftDriveMtrId = 12;
        public static final int frontLeftCanCoderId = 13;

        public static final int frontRightSteerMtrId = 21;
        public static final int frontRightDriveMtrId = 22;
        public static final int frontRightCanCoderId = 23;

        public static final int backLeftSteerMtrId = 14;
        public static final int backLeftDriveMtrId = 15;
        public static final int backLeftCanCoderId = 16;

        public static final int backRightSteerMtrId = 24;
        public static final int backRightDriveMtrId = 25;
        public static final int backRightCanCoderId = 26;
    }

    public static final class ControllerConstants {
        public static final int driverGamepadPort = 0; //Can be found in driver station

        public static final double joystickDeadband = 0.15; //Prevent small values from moving the robot (good for old controlers)

        public static final double triggerPressedThreshhold = 0.25;
    }
    
    public static final class DriveConstants {
        /**
         * The track width from wheel center to wheel center.
         */
        public static final double trackWidth = Units.inchesToMeters(22);

        /**
         * The track length from wheel center to wheel center.
         */
        public static final double wheelBase = Units.inchesToMeters(22);

        /**
         * The SwerveDriveKinematics used for control and odometry.
         */
        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right
            );

        /**
         * The gear reduction from the drive motor to the wheel.
         * 
         * The drive gear ratios for the different levels can be found from the chart at
         * swervedrivespecialties.com/products/mk41-swerve-module.
         */
        public static final double driveMtrGearReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);

        /**
         * The gear reduction from the steer motor to the wheel.
         */
        public static final double steerMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);
        
        /**
         * Values for our specific MK4i modules
         */
        public static final double wheelRadiusMeters = Units.inchesToMeters(2);
        public static final double wheelCircumferenceMeters = 2.0 * wheelRadiusMeters * Math.PI;
        public static final double driveBaseRadius = Units.inchesToMeters(14);

        public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
        public static final double driveMetersPerSecPerRPM = driveMetersPerEncRev / 60.0;

        public static final double steerRadiansPerEncRev = 2 * Math.PI * DriveConstants.steerMtrGearReduction;
        public static final double steerRadiansPerSecPerRPM = steerRadiansPerEncRev / 60;

        public static final double kFreeMetersPerSecond = 5820 * driveMetersPerSecPerRPM;

        public static final double steerMtrMaxSpeedRadPerSec = 2.0;
        public static final double steerMtrMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeedMetersPerSec = 5.0;

        /**
         * The rate the robot will spin with full Rot command.
         */
        public static final double maxTurnRateRadiansPerSec = 2.0 * Math.PI;

        /**
         * These offsets are only here for backwards compatibility.
         * Phoenix Tuner X allows for zeroing CANcoders.
         */
        public static final double frontLeftModOffset = Units.degreesToRadians(0); 
        public static final double frontRightModOffset = Units.degreesToRadians(0);
        public static final double backLeftModOffset = Units.degreesToRadians(0);
        public static final double backRightModOffset = Units.degreesToRadians(0); 

        // Some wheels would spin backwards
        public static final boolean frontLeftDriveInvert = false;
        public static final boolean frontRightDriveInvert = false;
        public static final boolean backLeftDriveInvert = false;
        public static final boolean backRightDriveInvert = true;

        // This is just-in-case
        public static final boolean frontLeftSteerInvert = false;
        public static final boolean frontRightSteerInvert = false;
        public static final boolean backLeftSteerInvert = false;
        public static final boolean backRightSteerInvert = false;

        public static final int driveCurrentLimitAmps = 40;
        
        public static final double drivekP = 0.005;
        public static final double driveI = 0;
        public static final double drivekD = 0.0;

        public static final double steerkP = 1;
        public static final double steerI = 0;
        public static final double steerkD = 0.0;

        public static final double ksVolts = 0.667;
        public static final double kvVoltSecsPerMeter = 2.44;
        public static final double kaVoltSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);
    }

    public static final class AutoConstants {
        /**
         * The default maximum speed of the robot in auto. Can be overridden by the FollowTrajectoryCmd Command.
         */
        public static final double maxVelMetersPerSec = 3.25;

        public static final double drivekP = 12.8;
        public static final double drivekD = 0.085;

        public static final PIDConstants driveConstants = new PIDConstants(drivekD, drivekD);

        public static final double rotkP = 1.27;
        public static final double rotkD = 0.5;

        public static final PIDConstants rotConstants = new PIDConstants(rotkP, rotkD);
    }
}
