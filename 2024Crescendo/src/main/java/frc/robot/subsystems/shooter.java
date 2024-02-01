// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private CANSparkMax m_leftShootMotor;
    private CANSparkMax m_rightShootMotor;

    public void motors() {
        m_leftShootMotor = new CANSparkMax(Constants.shooterConstants.leftShootSparkMaxCANID, MotorType.kBrushless);
        m_rightShootMotor = new CANSparkMax(Constants.shooterConstants.rightShootSparkMaxCANID, MotorType.kBrushless);
    }

  /** Creates a new Shooter. */
  public Shooter() {
    
    }
        public void shoot() {
        m_leftShootMotor.set(Constants.shooterConstants.lShootSpeed);
        m_rightShootMotor.set(Constants.shooterConstants.rShootSpeed);
        }
    public void stop() {
        m_leftShootMotor.set(Constants.shooterConstants.stop);
        m_rightShootMotor.set(Constants.shooterConstants.stop);

    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
