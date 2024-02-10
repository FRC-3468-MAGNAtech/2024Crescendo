// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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