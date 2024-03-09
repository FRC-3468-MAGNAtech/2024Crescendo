// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
	private SparkPIDController m_backPIDController;
	private SparkLimitSwitch m_reverseLimitSwitch;
	private RelativeEncoder m_leftEncoder;
	private double m_setPoint;
	private CANSparkMax m_leftArmNEO;
	private CANSparkMax m_rightArmNEO;


	/** Creates a new Climb. */
	public Climb() {
		m_leftArmNEO = new CANSparkMax(ClimbConstants.leftSparkMaxID,MotorType.kBrushless);
		m_rightArmNEO = new CANSparkMax(ClimbConstants.rightSparkMaxID, MotorType.kBrushless);

		m_leftArmNEO.setIdleMode(IdleMode.kBrake);
		m_rightArmNEO.setIdleMode(IdleMode.kBrake);

		m_rightArmNEO.follow(m_leftArmNEO);

		m_leftEncoder = m_leftArmNEO.getEncoder();

		m_reverseLimitSwitch = m_leftArmNEO.getReverseLimitSwitch(Type.kNormallyOpen);
		m_reverseLimitSwitch.enableLimitSwitch(true);

		m_backPIDController = m_leftArmNEO.getPIDController(); 

		m_backPIDController.setP(ClimbConstants.leftArmP);
		m_backPIDController.setI(ClimbConstants.leftArmI);
		m_backPIDController.setD(ClimbConstants.leftArmD);
		m_backPIDController.setIZone(ClimbConstants.leftArmIZone);
		m_backPIDController.setFF(ClimbConstants.leftArmFF);
		m_backPIDController.setOutputRange(ClimbConstants.leftArmMin,ClimbConstants.leftArmMax);

	}

	public void topLeftArmPID(){
		m_backPIDController.setReference(ClimbConstants.upPIDReference, ControlType.kPosition);
		m_setPoint = ClimbConstants.upPIDReference;
	}

	public void bottomLeftArmPID(){
		m_backPIDController.setReference(ClimbConstants.downPIDReference, ControlType.kPosition);
		m_setPoint = ClimbConstants.downPIDReference; 
	}

	public void setLeftAscendSpeed() {
		m_leftArmNEO.set(ClimbConstants.ascensionSpeed);
	}

	public void setLeftDescendSpeed() {
		m_leftArmNEO.set(ClimbConstants.descensionSpeed);    
	}

	public void stopLeftArm(){
		m_leftArmNEO.set(ClimbConstants.stopSpeed);
	}

	public boolean leftLimitSwitch() {
		return m_reverseLimitSwitch.isPressed();
	}

	public void setHome() {
		m_leftEncoder.setPosition(0.0);
	}

	public boolean isAtSetPoint() {
		return Math.abs(m_setPoint - m_leftEncoder.getPosition()) <= ClimbConstants.leftPIDTolerance;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putBoolean("LeftLimSwitch", leftLimitSwitch());
		SmartDashboard.putNumber("Left Arm Position", m_leftEncoder.getPosition());
	}
}