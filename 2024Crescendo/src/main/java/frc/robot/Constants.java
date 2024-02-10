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

 // 1:1 gear ratio
 // Wheel size: 2 in
 public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
  public static final class IntakeConstants {
  
    public static final int lowerIntakeMotorID = 0;
    public static final int upperIntakeMotorID = 1;
    public static final double lowerIntakeMotorSpeed = .2;
    public static final double upperIntakeMotorSpeed = .2;
    public static final boolean lowerIntakeMotorInvert = false;
    public static final boolean upperIntakeMotorInvert = false;
  }

  public static final class driveControllerConstants {
    public static final int primaryDriveControllerPort = 0;
    public static final int secondaryDriveControllerPort = 1;
    public static final int intakeButton = XboxController.Button.kX.value;
    public static final int speakerShooterButton = XboxController.Button.kA.value;
    public static final int ampShooterButton = XboxController.Button.kY.value;
    public static final int armRaiseButton = XboxController.Button.kRightBumper.value;
    public static final int armLowerButton = XboxController.Button.kLeftBumper.value;
  }
    

  public static final class shooterConstants {
    public static final int bottomShootSparkMaxCANID = 0;
    public static final int topShootSparkMaxCANID = 0;
    public static final double bottomShootSpeed = -0.2;
    public static final double topShootSpeed = 0.2;
    public static final double bottomAmpSpeed = -0.02;
    public static final double topAmpSpeed = 0.02;
    public static final double stop = 0.0;
  }    

  public static final class armConstants {
    public static final int leftArmSparkMaxCANID = 0;
    public static final int rightArmSparkMaxCANID = 0;
    public static final double raiseSpeed = 0.2;
    public static final double lowerSpeed = -0.2;
    public static final double stop = 0.0;
    public static final boolean armLimitTriggered = true;
  }
}
