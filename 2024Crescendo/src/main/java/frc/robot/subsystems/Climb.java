// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
	private CANSparkMax m_leftArmNEO;
	private RelativeEncoder m_leftEncoder;
	private SparkLimitSwitch m_leftReverseLimitSwitch;

	private CANSparkMax m_rightArmNEO;
	private RelativeEncoder m_rightEncoder;
	private SparkLimitSwitch m_rightReverseLimitSwitch;

	/** Creates a new Climb. */
	public Climb() {
		m_leftArmNEO = new CANSparkMax(ClimbConstants.leftSparkMaxID,MotorType.kBrushless);
		m_leftArmNEO.setIdleMode(IdleMode.kBrake);
		m_leftEncoder = m_leftArmNEO.getEncoder();
		m_leftReverseLimitSwitch = m_leftArmNEO.getReverseLimitSwitch(Type.kNormallyOpen);
		m_leftReverseLimitSwitch.enableLimitSwitch(true);

		m_rightArmNEO = new CANSparkMax(ClimbConstants.rightSparkMaxID, MotorType.kBrushless);
		m_rightArmNEO.setIdleMode(IdleMode.kBrake);
		m_rightEncoder = m_rightArmNEO.getEncoder();
		m_rightReverseLimitSwitch = m_rightArmNEO.getReverseLimitSwitch(Type.kNormallyOpen);
		m_rightReverseLimitSwitch.enableLimitSwitch(true);
	}

	public void setLeftAscendSpeed() {
		m_leftArmNEO.set(ClimbConstants.ascensionSpeed);
		m_rightArmNEO.set(ClimbConstants.ascensionSpeed);
	}

	public void setLeftDescendSpeed() {
		m_leftArmNEO.set(ClimbConstants.descensionSpeed);    
		m_rightArmNEO.set(ClimbConstants.descensionSpeed);  
	}

	public void stopArms(){
		m_leftArmNEO.set(ClimbConstants.stopSpeed);
		m_rightArmNEO.set(ClimbConstants.stopSpeed);
	}

	public boolean leftLimitSwitch() {
		return m_leftReverseLimitSwitch.isPressed();
	}

	public boolean rightLimitSwitch() {
		return m_rightReverseLimitSwitch.isPressed();
	}

	public void setHomeLeft() {
		m_leftEncoder.setPosition(0.0);
	}

	public void setHomeRight() {
		m_rightEncoder.setPosition(0.0);
	}

	public double getEncoderValue() {
		return m_leftEncoder.getPosition();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putBoolean("LeftLimSwitch", leftLimitSwitch());
		SmartDashboard.putNumber("Left Arm Position", m_leftEncoder.getPosition());
		SmartDashboard.putNumber("ClimbPosition", getEncoderValue());
		if (leftLimitSwitch())
			setHomeLeft();
		if (rightLimitSwitch())
			setHomeRight();
	}
}