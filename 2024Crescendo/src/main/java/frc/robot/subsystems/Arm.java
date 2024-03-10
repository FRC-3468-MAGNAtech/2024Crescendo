// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.armConstants;

public class Arm extends SubsystemBase {
	private CANSparkMax m_rightRaiseMotor;
	private SparkPIDController m_PIDController;
	private RelativeEncoder m_rEncoder;
	private AbsoluteEncoder m_Encoder;
	private SparkLimitSwitch m_bottemLimit;

	
	public Arm() {
		m_rightRaiseMotor = new CANSparkMax(Constants.armConstants.rightArmSparkMaxCANID, MotorType.kBrushless);
		m_rEncoder = m_rightRaiseMotor.getEncoder();
		m_Encoder = m_rightRaiseMotor.getAbsoluteEncoder();
		m_bottemLimit = m_rightRaiseMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_bottemLimit.enableLimitSwitch(true);
		m_rightRaiseMotor.setInverted(true);

		m_PIDController = m_rightRaiseMotor.getPIDController();

		m_PIDController.setP(armConstants.ArmP);
		m_PIDController.setI(armConstants.ArmI);
		m_PIDController.setD(armConstants.ArmD);
		m_PIDController.setIZone(armConstants.ArmIZone);
		m_PIDController.setFF(armConstants.ArmFF);
		m_PIDController.setOutputRange(armConstants.ArmMin, armConstants.ArmMax);
	}

	public void raise() {
		m_rightRaiseMotor.set(Constants.armConstants.raiseSpeed);
	}

	public void pointRaise() {
		m_PIDController.setReference(armConstants.upPIDReference, ControlType.kPosition);
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

	public void resetEncoder() {
		if (m_bottemLimit.isPressed()){
			m_rEncoder.setPosition(0.0);
		}
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Arm Current", m_rightRaiseMotor.getOutputCurrent());
		SmartDashboard.putNumber("ArmPosition", m_Encoder.getPosition());
		resetEncoder();
	}
}
