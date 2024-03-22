// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Camera extends SubsystemBase {

	 
	 
	 
	/** Creates a new Camera. */
	public Camera() {

	}

	public static double getArea() {
		double targetOffSetAngle_vertical = LimelightHelpers.getTA(LimelightConstants.llTags);
		return -targetOffSetAngle_vertical + 0.54;
		
	}
	
	public static double getTZ() {
		return -LimelightHelpers.getCameraPose3d_TargetSpace(LimelightConstants.llTags).getZ();
	}



	

	@Override
	public void periodic() { 
		// This method will be called once per scheduler run
	}
}
