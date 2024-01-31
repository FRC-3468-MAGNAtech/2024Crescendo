// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*Function: Move two arms to multple positions
* ShooterArm - moves arm from home position to Amp, Speaker, or Whatever the last position was
 * MuscleUparm - moves the muscle up arm from home position, to extended, to lifted, back to home
 * 
 */

 /*ShooterArm - 
  * Hardware - 2 NEO brushless 1Gear ratio
  Variables:
  ShooterArmGearRatio (double)
  ShooterMotor1 (int)
  ShooterMotor2 (int)
  ShooterMotorSpeed(double)
  
  */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbJB extends SubsystemBase {
  /** Creates a new ClimbJB. */
  public ClimbJB() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
