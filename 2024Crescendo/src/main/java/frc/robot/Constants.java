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
    public static final int ClimbDescendButton = XboxController.Axis.kLeftTrigger.value;
    public static final int ClimbAscendButton = XboxController.Axis.kRightTrigger.value;
    public static final int ClimbHomeButton = XboxController.Button.kA.value;
  }
  public static final class ClimbConstants {
    public static final int leftSparkMaxID = 0;
    public static final int rightSparkMaxID = 0;
    public static final double ascensionSpeed = 0.25;
    public static final double descensionSpeed = -0.25;
    public static final double stopSpeed = 0.0;
    public static final double leftArmP = 3.0;
    public static final double leftArmI = 0.0;
    public static final double leftArmD = 0.0;
    public static final double leftArmIZone = 0.0;
    public static final double leftArmFF = 0.0;
    public static final double leftArmMin = -0.5;
    public static final double leftArmMax = 1.0;
    public static final double upPIDReference = 85.0;
    public static final double downPIDReference = 7.0;
    public static final double leftPIDTolerance = 0.01;
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

  public static final class IntakeConstants {
    public static final int intakeMotorID = 0;
    public static final double intakeMotorForward = .2;
    public static final double intakeMotorReverse = -0.2;
    public static final boolean intakeMotorInvert = false;
  }  

  public static final class shooterConstants {
    public static final int bottomShootSparkMaxCANID = 0;
    public static final int topShootSparkMaxCANID = 1;
    public static final double bottomShootSpeed = -0.2;
    public static final double topShootSpeed = 0.2;
    public static final double bottomAmpSpeed = -0.02;
    public static final double topAmpSpeed = 0.02;
    public static final double shooterP = 0.1;
    public static final double shooterI = 0.1;
    public static final double shooterD = 0.1;
    public static final double shooterIZone = 0.1;
    public static final double shooterFF = 0.1;
    public static final double shooterMin = -0.5;
    public static final double shooterMax = 1.0;
    public static final double upPIDReference = 85.0;
  }    

  public static final class armConstants {
    public static final int leftArmSparkMaxCANID = 3;
    public static final int rightArmSparkMaxCANID = 4;
    public static final double raiseSpeed = 0.2;
    public static final double lowerSpeed = -0.2;
    public static final boolean armLimitTriggered = true;
    public static final double ArmP = 0.1;
    public static final double ArmI = 0.1;
    public static final double ArmD = 0.1;
    public static final double ArmIZone = 0.1;
    public static final double ArmFF = 0.1;
    public static final double ArmMin = -0.5;
    public static final double ArmMax = 1.0;
    public static final double upPIDReference = 85.0;
  }
}
