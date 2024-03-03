// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.armConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax m_rightRaiseMotor;
  private SparkPIDController m_PIDController;

  
  public Arm() {
    m_rightRaiseMotor = new CANSparkMax(Constants.armConstants.rightArmSparkMaxCANID, MotorType.kBrushless);

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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
