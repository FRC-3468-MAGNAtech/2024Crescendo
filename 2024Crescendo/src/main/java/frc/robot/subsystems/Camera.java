// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Camera extends SubsystemBase {

	public static double targetOffSetAngle_vertical = LimelightHelpers.getTY(LimelightConstants.llTags);
	public static double llDegrees = 115;
	public static double llHeight = 11.75;
	public static double goalHieght = 57;
	public static double angleToGoalDegrees = llDegrees + targetOffSetAngle_vertical;
	public static double angleToGoalRadians = angleToGoalDegrees * (Math.PI/180);
	public static double distanceFromllToGoalInches = (goalHieght - llHeight) / Math.tan(angleToGoalRadians);
	/** Creates a new Camera. */
	public Camera() {
		
	}

	public static double getDistance() {
		return distanceFromllToGoalInches;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
