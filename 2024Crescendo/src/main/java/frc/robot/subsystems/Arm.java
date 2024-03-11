// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.shooterConstants;

public class Arm extends SubsystemBase {
	public double ReferenceAngle = 0.35;
	private CANSparkMax m_rightRaiseMotor;
	private SparkPIDController m_PIDController;
	private AbsoluteEncoder m_Encoder;
	private SparkLimitSwitch m_bottomLimit;

	
	public Arm() {
		m_rightRaiseMotor = new CANSparkMax(Constants.armConstants.rightArmSparkMaxCANID, MotorType.kBrushless);
		m_Encoder = m_rightRaiseMotor.getAbsoluteEncoder();
		m_bottomLimit = m_rightRaiseMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_bottomLimit.enableLimitSwitch(true);
		m_rightRaiseMotor.setInverted(true);

		m_PIDController = m_rightRaiseMotor.getPIDController();
		m_PIDController.setFeedbackDevice(m_Encoder);
		

		m_PIDController.setP(armConstants.ArmP);
		m_PIDController.setI(armConstants.ArmI);
		m_PIDController.setD(armConstants.ArmD);
		m_PIDController.setIZone(armConstants.ArmIZone);
		m_PIDController.setFF(armConstants.ArmFF);
		m_PIDController.setOutputRange(armConstants.ArmMin, armConstants.ArmMax);

		//m_rEncoder.setPositionConversionFactor(armConstants.armGearRatio);
		
	}
	public boolean isParked() {
		return m_bottomLimit.isPressed();
	}
	
	public boolean isAtSetPoint() {
		return m_PIDController.getPositionPIDWrappingMinInput() == ReferenceAngle;
	}

	public void moveToAngle(double angle) {
		m_PIDController.setReference(angle, ControlType.kPosition);
	}

	public void raise() {
		m_rightRaiseMotor.set(Constants.armConstants.raiseSpeed);
	}

	public void pointMove(double angle) {
		m_PIDController.setReference(armConstants.shooterEquationA * Math.log(armConstants.shooterEquationB * (Camera.distenceFromllToGoalInches/12)) + armConstants.shooterEquationC , ControlType.kPosition);
	}

	public void lower(){
		m_rightRaiseMotor.set(Constants.armConstants.lowerSpeed);
	}

	public void pointLower() {
		m_PIDController.setReference(0.0, ControlType.kPosition);
	}

	public void stop(){
		m_rightRaiseMotor.set(0);
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Arm Set Speed", m_rightRaiseMotor.get());
		SmartDashboard.putNumber("ArmPositionA", m_Encoder.getPosition());
	}
}
