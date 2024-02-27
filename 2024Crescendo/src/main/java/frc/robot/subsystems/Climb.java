// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LeftArmConstants;
import frc.robot.Constants.RightArmConstants;

public class Climb extends SubsystemBase {

    private SparkPIDController m_backPIDController;
  private SparkLimitSwitch m_reverseLimitSwitch;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private double m_setPoint;
  private CANSparkMax m_leftArmNEO;
  private CANSparkMax m_rightArmNEO;
  
  /** Creates a new Climb. */
  public Climb() {

      m_leftArmNEO = new CANSparkMax(LeftArmConstants.leftSparkMaxID,MotorType.kBrushless);
      m_rightArmNEO = new CANSparkMax(RightArmConstants.rightSparkMaxID, MotorType.kBrushless);

      m_rightArmNEO.follow(m_leftArmNEO);

      m_leftEncoder = m_leftArmNEO.getEncoder();
      m_rightEncoder = m_rightArmNEO.getEncoder();


      m_reverseLimitSwitch = m_leftArmNEO.getReverseLimitSwitch(Type.kNormallyOpen);
      m_reverseLimitSwitch.enableLimitSwitch(true);

      m_backPIDController = m_leftArmNEO.getPIDController(); 

      m_backPIDController.setP(LeftArmConstants.leftArmP);
      m_backPIDController.setI(LeftArmConstants.leftArmI);
      m_backPIDController.setD(LeftArmConstants.leftArmD);
      m_backPIDController.setIZone(LeftArmConstants.leftArmIZone);
      m_backPIDController.setFF(LeftArmConstants.leftArmFF);
      m_backPIDController.setOutputRange(LeftArmConstants.leftArmMin,LeftArmConstants.leftArmMax);

      m_reverseLimitSwitch = m_rightArmNEO.getReverseLimitSwitch(Type.kNormallyOpen);
      m_reverseLimitSwitch.enableLimitSwitch(true);
  
      m_backPIDController = m_rightArmNEO.getPIDController(); 
  
      m_backPIDController.setP(RightArmConstants.rightArmP);
      m_backPIDController.setI(RightArmConstants.rightArmI);
      m_backPIDController.setD(RightArmConstants.rightArmD);
      m_backPIDController.setIZone(RightArmConstants.rightArmIZone);
      m_backPIDController.setFF(RightArmConstants.rightArmFF);
      m_backPIDController.setOutputRange(RightArmConstants.rightArmMin,RightArmConstants.rightArmMax);
  

      m_leftArmNEO.setIdleMode(IdleMode.kBrake);

      m_leftArmNEO.burnFlash();
    }

    public void topLeftArmPID(){
      m_backPIDController.setReference(LeftArmConstants.upPIDReference, ControlType.kPosition);
      m_setPoint = LeftArmConstants.upPIDReference;
    }

    public void bottomLeftArmPID(){
      m_backPIDController.setReference(LeftArmConstants.downPIDReference, ControlType.kPosition);
      m_setPoint = LeftArmConstants.downPIDReference; 
    }

    public void setLeftAscendSpeed() {
      m_leftArmNEO.set(LeftArmConstants.ascensionSpeed);
    }

    public void setLeftDescendSpeed() {
      m_leftArmNEO.set(LeftArmConstants.descensionSpeed);    
    }

    public void stopLeftArm(){
      m_leftArmNEO.set(LeftArmConstants.stopSpeed);
    }

    public boolean leftLimitSwitch() {
      return m_reverseLimitSwitch.isPressed();
    }

    public void setHome() {
      m_leftEncoder.setPosition(0.0);
    }

    public boolean isAtSetPoint() {
      return Math.abs(m_setPoint - m_leftEncoder.getPosition()) <= LeftArmConstants.leftPIDTolerance;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putBoolean("LeftLimSwitch", leftLimitSwitch());
    SmartDashboard.putNumber("Left Arm Position", m_leftEncoder.getPosition());
  }
}

 