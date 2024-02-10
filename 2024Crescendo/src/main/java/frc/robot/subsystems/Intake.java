// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// The intake will pick up the ring and hold in in place using 1 motor.  it will use gears and belts to move 3 rails of wheels
// it have a gear ratio.  The motor is a RevRobotics NEO brushless motor controlled by CANBUS by way of a SPARKMax controller.  
// The motor will run at three different speeds.  the first speed will be intake, the second will be chamber speed.  In order
// to fix jamming, we will need to be able to reverse and sens the position of the note.  We will use 2 photo eye sensors to 
// identify position of the note.  the first sensor will tell the motor to stop when it is triggered.  In case the note goes 
// too far, the second sensor will trigger and have the motor go in reverse until the sensor is no longer triggered. 

// INT (integer) whole number - or +
// BOOLEAN true/false, yes/no, 0-1
// STRING letters, numbers, or charater that has no value 
// DOUBLE
// 


// constants
// GearRatio
// WheelSize 
// 


//VARIABLES
// MotorSpeed double
// PhotoEye1 boolean
// PhotoEye2 boolean
// MotorOnDuration double
// interrupt boolean

// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  
  // Intake Motor Controllers
  private CANSparkMax m_upperIntakeBar; // NEO 550 motor  
  private RelativeEncoder m_upperIntakeBarEncoder;  // NEO 550 encoder
  private double upperIntakeBarRPM;


  /** Subsystem for controlling the Intake */
  public Intake() {
    // Instantiate the Intake motor controllers
    m_upperIntakeBar = new CANSparkMax(IntakeConstants.upperIntakeMotorID, MotorType.kBrushless);


    // Reverse some of the motors if needed
    m_upperIntakeBar.setInverted(IntakeConstants.upperIntakeMotorInvert);
    m_upperIntakeBarEncoder = m_upperIntakeBar.getEncoder();


    SmartDashboard.putNumber("Upper Intake Bar Speed", IntakeConstants.upperIntakeMotorSpeed);
  }

  /* Set power to the intake motors */
  public void setPower(double upperPower) {
    m_upperIntakeBar.set(upperPower);
    
  }
  public void stop() {
    m_upperIntakeBar.set(0);
  }

  /* Read the speed of the intake motors */
  public double getLowerIntakeBarRPM() {
    return upperIntakeBarRPM;
  }
  

  @Override
  public void periodic() {
    
    upperIntakeBarRPM = m_upperIntakeBarEncoder.getVelocity();

    // Add intake bar RPM readingss to SmartDashboard for the sake of datalogging
    
    SmartDashboard.putNumber("Upper Intake Bar RPM", upperIntakeBarRPM);
 }
}