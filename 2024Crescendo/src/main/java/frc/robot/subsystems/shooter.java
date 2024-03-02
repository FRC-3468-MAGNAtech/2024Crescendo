// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_bottomShootMotor;
  private CANSparkMax m_topShootMotor;
  private SparkPIDController m_PIDController;


  /** Creates a new Shooter. */
  public Shooter() {
    m_bottomShootMotor = new CANSparkMax(Constants.shooterConstants.bottomShootSparkMaxCANID, MotorType.kBrushless);
    m_topShootMotor = new CANSparkMax(Constants.shooterConstants.topShootSparkMaxCANID, MotorType.kBrushless);

    m_PIDController = m_topShootMotor.getPIDController();

    m_PIDController.setP(shooterConstants.shooterP);
    m_PIDController.setI(shooterConstants.shooterI);
    m_PIDController.setD(shooterConstants.shooterD);
    m_PIDController.setIZone(shooterConstants.shooterIZone);
    m_PIDController.setFF(shooterConstants.shooterFF);
    m_PIDController.setOutputRange(shooterConstants.shooterMin, shooterConstants.shooterMax);
  }

  public void shoot() {
    m_bottomShootMotor.set(Constants.shooterConstants.bottomShootSpeed);
    m_topShootMotor.set(Constants.shooterConstants.topShootSpeed);
  }

  public void shootDistance() {
    m_PIDController.setReference(shooterConstants.upPIDReference, ControlType.kVelocity);
  }

  public void amp() {
    m_bottomShootMotor.set(Constants.shooterConstants.bottomAmpSpeed);
    m_topShootMotor.set(Constants.shooterConstants.topAmpSpeed);
  }
  
  public void stop() {
    m_bottomShootMotor.set(0);
    m_topShootMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
