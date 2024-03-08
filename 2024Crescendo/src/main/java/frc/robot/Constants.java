// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {


	public static final class SecondDriveControllerConstants{
		public static final int intakeButton = XboxController.Button.kA.value;
		public static final int extakeButton = XboxController.Button.kY.value;
		public static final int shootButton = XboxController.Button.kB.value;
		public static final int shootArmRaiseButton = XboxController.Button.kRightBumper.value;
		public static final int shootArmLowerButton = XboxController.Button.kLeftBumper.value;
		public static final int climbUpBotton = XboxController.Axis.kRightTrigger.value;
		public static final int climbDownButton = XboxController.Axis.kLeftTrigger.value;
	}

	// These IDs are actually backwards because during rotation, the wheels would be an x
	public static final class CANDevices {
		public static final int powerDistributionHubId = 1;
		public static final int pneumaticHubId = 3;

		//Pigeon2 ID
		public static final int imuId = 2;

		public static final int frontLeftSteerMtrId = 21;
		public static final int frontLeftDriveMtrId = 22;
		public static final int frontLeftCanCoderId = 23;

		public static final int frontRightSteerMtrId = 11;
		public static final int frontRightDriveMtrId = 12;
		public static final int frontRightCanCoderId = 13;

		public static final int backLeftSteerMtrId = 24;
		public static final int backLeftDriveMtrId = 25;
		public static final int backLeftCanCoderId = 26;

		public static final int backRightSteerMtrId = 14;
		public static final int backRightDriveMtrId = 15;
		public static final int backRightCanCoderId = 16;
	}

	/**
	 * LED Light display on robot for statuses
	 */
	public static final class LEDConstants {
		public static final int LEDPower = 0;
		public static final int LEDRed = 1;
		public static final int LEDGreen = 2;
		public static final int LEDBlue = 3;
		public static final int LEDWhite = 4;
		
	}

	public static final class ControllerConstants {
		public static final int driverGamepadPort = 0; //Can be found in driver station

		public static final double joystickDeadband = 0.15; //Prevent small values from moving the robot (good for old controlers)

		public static final double triggerPressedThreshhold = 0.25;
	}

	public static final class LimelightConstants {
		/**
		 * Limelight Strings
		 */
		public static final String llTags = "limelight-tags";
		public static final String llNotes = "limelight-notes";

		public static final double driveKP = 0.025;
		public static final double rotateKP = 0.03;
		public static final PIDController llPIDctrlDrive = new PIDController(driveKP, 0, 0);
		public static final PIDController llPIDctrlRotate = new PIDController(rotateKP, 0, 0);
	}
	
	public static final class DriveConstants {
	/**
	 * The track width from wheel center to wheel center.
	 */
	public static final double trackWidth = Units.inchesToMeters(24);

	/**
	 * The track length from wheel center to wheel center.
	 */
	public static final double wheelBase = Units.inchesToMeters(24.5);

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

		public static final double maxDriveSpeedMetersPerSec = 3.0;

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
		public static final boolean frontLeftSteerInvert = true;
		public static final boolean frontRightSteerInvert = true;
		public static final boolean backLeftSteerInvert = true;
		public static final boolean backRightSteerInvert = true;

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

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
		public static final int ClimbDescendButton = XboxController.Axis.kLeftTrigger.value;
		public static final int ClimbAscendButton = XboxController.Axis.kRightTrigger.value;
		public static final int ClimbHomeButton = XboxController.Button.kA.value;
	}
	
	public static final class ClimbConstants {
		public static final int leftSparkMaxID = 6;
		public static final int rightSparkMaxID = 5;
		public static final double ascensionSpeed = 0.7;
		public static final double descensionSpeed = -0.5;
		public static final double stopSpeed = 0.0;
		public static final double leftArmP = 3.0;
		public static final double leftArmI = 0.0;
		public static final double leftArmD = 0.0;
		public static final double leftArmIZone = 0.0;
		public static final double leftArmFF = 0.0;
		public static final double leftArmMin = -0.5;
		public static final double leftArmMax = 1.0;
		public static final double upPIDReference = 85.0;
		public static final double downPIDReference = 7.0;
		public static final double leftPIDTolerance = 0.01;
	}
	
	public static final class driveControllerConstants {
		public static final int primaryDriveControllerPort = 0;
		public static final int secondaryDriveControllerPort = 1;
		public static final int intakeButton = XboxController.Button.kX.value;
		public static final int speakerShooterButton = XboxController.Button.kA.value;
		public static final int ampShooterButton = XboxController.Button.kY.value;
		public static final int armRaiseButton = XboxController.Button.kRightBumper.value;
		public static final int armLowerButton = XboxController.Button.kLeftBumper.value;
	}

	public static final class IntakeConstants {
		public static final int intakeMotorID = 7;
		public static final double intakeMotorForward = -.2;
		public static final double intakeMotorReverse = 0.1;
		public static final boolean intakeMotorInvert = false;
	}  

	public static final class shooterConstants {
		public static final int bottomShootSparkMaxCANID = 8;
		public static final int topShootSparkMaxCANID = 9;
		public static final double bottomShootSpeed = 0.7;
		public static final double topShootSpeed = -0.7;
		public static final double bottomAmpSpeed = -0.02;
		public static final double topAmpSpeed = 0.02;
		public static final double shooterP = 0.1;
		public static final double shooterI = 0.1;
		public static final double shooterD = 0.1;
		public static final double shooterIZone = 0.1;
		public static final double shooterFF = 0.1;
		public static final double shooterMin = -0.5;
		public static final double shooterMax = 1.0;
		public static final double upPIDReference = 85.0;
	}    

	public static final class armConstants {
		public static final int rightArmSparkMaxCANID = 4;
		public static final double raiseSpeed = 0.5;
		public static final double lowerSpeed = -0.5;
		public static final boolean armLimitTriggered = true;
		public static final double ArmP = 0.1;
		public static final double ArmI = 0.1;
		public static final double ArmD = 0.1;
		public static final double ArmIZone = 0.1;
		public static final double ArmFF = 0.1;
		public static final double ArmMin = -0.5;
		public static final double ArmMax = 1.0;
		public static final double upPIDReference = 85.0;
	}
}
