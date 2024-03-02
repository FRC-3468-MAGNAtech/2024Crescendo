// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.*;

import javax.sound.midi.MidiChannel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  
  // Intake Motor Controllers
  private CANSparkMax m_intakeMotor; 
  private RelativeEncoder m_intakeEncoder; 


  public Intake() {
    // Instantiate the Intake motor controllers
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

    // Reverse some of the motors if needed
    m_intakeEncoder = m_intakeMotor.getEncoder();
  }
  
  public void intake() {
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
 }
}