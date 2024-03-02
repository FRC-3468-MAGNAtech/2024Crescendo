// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

    private CANSparkMax m_leftRaiseMotor;
    private CANSparkMax m_rightRaiseMotor;

    public void motors() {
        m_leftRaiseMotor = new CANSparkMax(Constants.armConstants.leftArmSparkMaxCANID, MotorType.kBrushless);
        m_rightRaiseMotor = new CANSparkMax(Constants.armConstants.rightArmSparkMaxCANID, MotorType.kBrushless);

        m_leftRaiseMotor.follow(m_rightRaiseMotor);
        
    }

  public Arm() {}

  public void raise() {
    m_rightRaiseMotor.set(Constants.armConstants.raiseSpeed);
  }

  public void stop(){
    m_rightRaiseMotor.set(Constants.armConstants.stop);
  }
  
  public void lower(){
    m_rightRaiseMotor.set(Constants.armConstants.lowerSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
