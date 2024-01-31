// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
}
