// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Class to represent and handle a swerve module
 * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
 * for both steeration and linear movement
 */
public class SwerveModule extends SubsystemBase {

	private final CANSparkMax driveMtr;
	private final CANSparkMax steerMtr;

	private final RelativeEncoder driveEnc;
	private final RelativeEncoder steerEnc;

	public final CANcoder canCoder;

	// Absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on.
	private final Rotation2d offset;

	private final SparkPIDController steerController;
	private final SparkPIDController driveController;

	/**
	 * Constructs a new SwerveModule.
	 * 
	 * <p>SwerveModule represents and handles a swerve module.
	 * 
	 * @param driveMtrId CAN ID of the NEO drive motor.
	 * @param steerMtrId CAN ID of the NEO steer motor.
	 * @param canCoderId CAN ID of the CANCoder.
	 * @param measuredOffsetRadians Offset of CANCoder reading from forward.
	 */
	public SwerveModule(int driveMtrId, int steerMtrId, int canCoderId, double measuredOffsetRadians, boolean invertDrive, boolean invertSteer) {
		//CANcoder data and offset
		canCoder = new CANcoder(canCoderId);
		offset = new Rotation2d(measuredOffsetRadians);

		//drive motor data
		driveMtr = new CANSparkMax(driveMtrId, MotorType.kBrushless);
		driveEnc = driveMtr.getEncoder();
		driveMtr.setIdleMode(IdleMode.kBrake);
		driveMtr.setSmartCurrentLimit(DriveConstants.driveCurrentLimitAmps);
		driveMtr.setInverted(invertDrive);

		driveController = driveMtr.getPIDController();
		driveController.setP(DriveConstants.drivekP);
		driveController.setD(DriveConstants.drivekD);

		//set the output of the drive encoder to be in radians for linear measurement
		driveEnc.setPositionConversionFactor(DriveConstants.driveMetersPerEncRev);

		//set the output of the drive encoder to be in radians per second for velocity measurement
		driveEnc.setVelocityConversionFactor(DriveConstants.driveMetersPerSecPerRPM);

		//set the drive encoder position to zero        
		driveEnc.setPosition(0);

		//steer motor data
		steerMtr = new CANSparkMax(steerMtrId, MotorType.kBrushless);
		steerEnc = steerMtr.getEncoder();
		steerMtr.setIdleMode(IdleMode.kCoast);
		steerMtr.setInverted(invertSteer);
		
		steerController = steerMtr.getPIDController();
		steerController.setP(DriveConstants.steerkP);
		steerController.setD(DriveConstants.steerkD);

		//set the output of the steering encoder to be in radians
		steerEnc.setPositionConversionFactor(DriveConstants.steerRadiansPerEncRev);

		//set the output to radians per second for velocity output
		steerEnc.setVelocityConversionFactor(DriveConstants.steerRadiansPerSecPerRPM);

		// Initializes the steer encoder position to the CANCoder position, accounting for an offset if any
		steerEnc.setPosition(getCanCoderAngle().getRadians() - offset.getRadians());

	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveEnc.getPosition(), getSteerEncAngle());
	}

	/**
	 * Resets the distance traveled by the module to zero.
	 */
	public void resetDriveDistance() {
		driveEnc.setPosition(0.0);
	}

	/**
	 * Returns the current drive distance of the module.
	 * 
	 * @return The current drive distance of the module.
	 */
	public double getDriveDistanceMeters() {

		return driveEnc.getPosition();

	}
	
	/**
	 * Returns the current absolute angle of the module from the CANCoder.
	 * This measurement does not account for offset.
	 * It is preferred to use the method getSteerEncAngle().
	 * 
	 * @return The value of the CANCoder.
	 */
	public Rotation2d getCanCoderAngle() {
		return new Rotation2d(Units.rotationsToRadians(
			canCoder.getAbsolutePosition().getValueAsDouble()
			));
	}

	/**
	 * Returns the current absolute angle of the module from the steer motor encoder.
	 * This measurement accounts for offset.
	 * 
	 * @return The current absolute angle of the module.
	 */
	public Rotation2d getSteerEncAngle() {
		return new Rotation2d(steerEnc.getPosition());
	}

	/**
	 * Returns the current velocity of the module from the drive motor encoder.
	 * 
	 * @return The current velocity of the module in meters per second.
	 */
	public double getVelocityMetersPerSec() {
		return driveEnc.getVelocity();
	}

	/**
	 * Calculates the angle motor setpoint based on the desired angle and the current angle measurement.
	 * 
	 * @param targetAngle The desired angle to set the module in radians.
	 * @param currentAngle The current angle of the module in radians.
	 * 
	 * @return The adjusted target angle for the module in radians.
	 */
	public double calculateAdjustedAngle(double targetAngle, double currentAngle) {
		double modAngle = currentAngle % (2.0 * Math.PI);

		if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
		
		double newTarget = targetAngle + currentAngle - modAngle;

		if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
		else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

		return newTarget;
	}

	/**
	 * Sets the desired state of the swerve module and optimizes it.
	 * <p>If closed-loop, uses PID and a feedforward to control the speed.
	 * If open-loop, sets the speed to a percentage. Open-loop control should
	 * only be used if running an autonomour trajectory.
	 *
	 * @param desiredState Object that holds a desired linear and steerational setpoint.
	 * @param isOpenLoop True if the velocity control is open- or closed-loop.
	 */
	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
		
		// Optimizes speed and angle to minimize change in heading
		// (e.g. module turns 1 degree and reverses drive direction to get from 90 degrees to -89 degrees)
		desiredState = SwerveModuleState.optimize(desiredState, getSteerEncAngle());

		steerController.setReference(
			calculateAdjustedAngle(
				desiredState.angle.getRadians(),
				getSteerEncAngle().getRadians()),
			ControlType.kPosition
		);

		if(isOpenLoop) 
			driveMtr.set(desiredState.speedMetersPerSecond / DriveConstants.kFreeMetersPerSecond);
		else {
			double speedMetersPerSecond = desiredState.speedMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec;

			driveController.setReference(
				speedMetersPerSecond,
				ControlType.kVelocity,
				0, 
				DriveConstants.driveFF.calculate(speedMetersPerSecond)
			);
		}
	}

	public void setDriveCurrentLimit(int amps) {
		driveMtr.setSmartCurrentLimit(amps);
	}
}