// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static final class driveControllerConstants {
    public static final int primaryDriveControllerPort = 0;
    public static final int secondaryDriveControllerPort = 1;
    public static final int intakeButton = XboxController.Button.kX.value;
    public static final int speakerShooterButton = XboxController.Button.kA.value;
    public static final int ampShooterButton = XboxController.Button.kY.value;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorID = 0;
    public static final double intakeMotorForward = .2;
    public static final double intakeMotorReverse = -0.2;
    public static final boolean intakeMotorInvert = false;
  }  

  public static final class shooterConstants {
    public static final int bottomShootSparkMaxCANID = 0;
    public static final int topShootSparkMaxCANID = 0;
    public static final double bottomShootSpeed = -1.0;
    public static final double topShootSpeed = 1.0;
    public static final double bottomAmpSpeed = -0.5;
    public static final double topAmpSpeed = 0.5;
    public static final double stop = 0.0;
  }    
}
