// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private CANSparkMax m_bottomShootMotor;
    private CANSparkMax m_topShootMotor;

    public void motors() {
        m_bottomShootMotor = new CANSparkMax(Constants.shooterConstants.bottomShootSparkMaxCANID, MotorType.kBrushless);
        m_topShootMotor = new CANSparkMax(Constants.shooterConstants.topShootSparkMaxCANID, MotorType.kBrushless);
    }

  /** Creates a new Shooter. */
  public Shooter() {
    
    }
    public void shoot() {
      m_bottomShootMotor.set(Constants.shooterConstants.bottomShootSpeed);
      m_topShootMotor.set(Constants.shooterConstants.topShootSpeed);
    }
    public void amp() {
      m_bottomShootMotor.set(Constants.shooterConstants.bottomAmpSpeed);
      m_topShootMotor.set(Constants.shooterConstants.topAmpSpeed);
    }
    public void stop() {
        m_bottomShootMotor.set(Constants.shooterConstants.stop);
        m_topShootMotor.set(Constants.shooterConstants.stop);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
