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

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Intake() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
