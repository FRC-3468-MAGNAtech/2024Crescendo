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
  public static final class LeftArmConstants {
    public static final int leftSparkMaxID = 0;
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
 public static final class RightArmConstants {
        public static final int rightSparkMaxID = 0;
        public static final double ascensionSpeed = 0.25;
        public static final double descensionSpeed = -0.25;
        public static final double stopSpeed = 0.0;
        public static final double rightArmP = 3.0;
        public static final double rightArmI = 0.0;
        public static final double rightArmD = 0.0;
        public static final double rightArmIZone = 0.0;
        public static final double rightArmFF = 0.0;
        public static final double rightArmMin = -0.5;
        public static final double rightArmMax = 1.0;
        public static final double upPIDReference = 85.0;
        public static final double downPIDReference = 7.0;
        public static final double rightPIDTolerance = 0.01;
    }
}
