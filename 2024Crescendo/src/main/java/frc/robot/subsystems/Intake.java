// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
	
	// Intake Motor Controllers
	private CANSparkMax m_intakeMotor; 
	private RelativeEncoder m_intakeEncoder; 

	private SparkPIDController m_pidcontroller;
	private DigitalInput intakeSensor;


	public Intake() {
		// Instantiate the Intake motor controllers
		m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
		m_intakeMotor.setIdleMode(IdleMode.kBrake);
		intakeSensor = new DigitalInput(IntakeConstants.intakeSensorID);
		

		// Reverse some of the motors if needed
		m_intakeEncoder = m_intakeMotor.getEncoder();
		m_intakeMotor.setInverted(true);

		m_pidcontroller = m_intakeMotor.getPIDController();
		m_pidcontroller.setFeedbackDevice(m_intakeEncoder);
		m_pidcontroller.setP(IntakeConstants.intakeP);
		m_pidcontroller.setI(IntakeConstants.intakeI);
		m_pidcontroller.setD(IntakeConstants.intakeD);
		m_pidcontroller.setFF(IntakeConstants.intakeFF);
		m_pidcontroller.setIZone(IntakeConstants.intakeIZone);
		m_pidcontroller.setOutputRange(0, IntakeConstants.intakeMax);
	}

	public boolean getIntakeSensor() {
		return !intakeSensor.get();
	}
	
	public void intake() {
		m_pidcontroller.setReference(IntakeConstants.intakeRPM, ControlType.kVelocity);
	}

	public void intakeOpen() {
		m_intakeMotor.set(IntakeConstants.intakeMotorForward); 
	}

	public void extake(){
		m_intakeMotor.set(IntakeConstants.intakeMotorReverse);
	}

	public void stop() {
		m_intakeMotor.set(0);
	}
	
	public double getIntakeRPM() {
		return m_intakeEncoder.getVelocity();
	}
	
	
	@Override
	public void periodic() {    
		// Add intake bar RPM readingss to SmartDashboard for the sake of datalogging
		SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
		SmartDashboard.putBoolean("IntakeSensor", getIntakeSensor());
	}
}