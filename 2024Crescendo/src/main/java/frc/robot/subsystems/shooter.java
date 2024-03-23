// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
	private CANSparkMax m_bottomShootMotor;
	private CANSparkMax m_topShootMotor;
	private SparkPIDController m_PIDControllerTop;
	private SparkPIDController m_PIDControllerBottom;
	private RelativeEncoder topEncoder;


	/** Creates a new Shooter. */
	public Shooter() {
		m_bottomShootMotor = new CANSparkMax(Constants.shooterConstants.bottomShootSparkMaxCANID, MotorType.kBrushless);
		m_topShootMotor = new CANSparkMax(Constants.shooterConstants.topShootSparkMaxCANID, MotorType.kBrushless);
		topEncoder = m_topShootMotor.getEncoder();

		m_PIDControllerTop = m_topShootMotor.getPIDController();

		m_PIDControllerTop.setP(shooterConstants.shooterP);
		m_PIDControllerTop.setI(shooterConstants.shooterI);
		m_PIDControllerTop.setD(shooterConstants.shooterD);
		m_PIDControllerTop.setIZone(shooterConstants.shooterIZone);
		m_PIDControllerTop.setFF(shooterConstants.shooterFF);
		// m_PIDControllerTop.setOutputRange(shooterConstants.shooterMin, shooterConstants.shooterMax);

		m_PIDControllerBottom = m_topShootMotor.getPIDController();

		m_PIDControllerBottom.setP(shooterConstants.shooterP);
		m_PIDControllerBottom.setI(shooterConstants.shooterI);
		m_PIDControllerBottom.setD(shooterConstants.shooterD);
		m_PIDControllerBottom.setIZone(shooterConstants.shooterIZone);
		m_PIDControllerBottom.setFF(shooterConstants.shooterFF);
		// m_PIDControllerBottom.setOutputRange(shooterConstants.shooterMin, shooterConstants.shooterMax);
	}

	public void shoot() {
		m_bottomShootMotor.set(Constants.shooterConstants.bottomShootSpeed);
		m_topShootMotor.set(Constants.shooterConstants.topShootSpeed);
	}

	public void shootDistance() {
		m_PIDControllerTop.setReference(shooterConstants.upPIDReference, ControlType.kVelocity);
		m_PIDControllerBottom.setReference(shooterConstants.upPIDReference, ControlType.kVelocity);
	}

	public void amp() {
		m_bottomShootMotor.set(Constants.shooterConstants.bottomAmpSpeed);
		m_topShootMotor.set(Constants.shooterConstants.topAmpSpeed);
	}
	
	public void stop() {
		m_bottomShootMotor.set(0);
		m_topShootMotor.set(0);
	}

	public double getVelocity() {
		return topEncoder.getVelocity();	
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Top Shooter Velocity", getVelocity());
		SmartDashboard.putNumber("SetShooterSpeed", shooterConstants.bottomShootSpeed);
	}
}
