// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Camera extends SubsystemBase {

	public InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
	public InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
	 
	 
	/** Creates a new Camera. */
	public Camera() {
		speedMap.put(0.858, 0.55);
		speedMap.put(1.978, 0.55);
		speedMap.put(2.086, 0.65);
		speedMap.put(2.460, 0.65);
		speedMap.put(2.100, 0.65);
		speedMap.put(3.100, 0.90);
		speedMap.put(3.800, 0.85);
		speedMap.put(2.300, 0.85);
		speedMap.put(2.800, 0.80);

		angleMap.put(0.858, 0.4048);
		angleMap.put(1.978, 0.4306);
		angleMap.put(2.086, 0.4500);
		angleMap.put(2.460, 0.4400);
		angleMap.put(2.100, 0.4500);
		angleMap.put(3.100, 0.4700);
		angleMap.put(3.800, 0.4700);
		angleMap.put(4.900, 0.4780);
		angleMap.put(2.300, 0.4500);
		angleMap.put(1.730, 0.4565);
		angleMap.put(0.770, 0.4126);
		angleMap.put(2.530, 0.4560);
		angleMap.put(2.800, 0.4500);
		angleMap.put(4.190, 0.4826);
		angleMap.put(4.350, 0.4835);

	}
	
	public static double getTZ() {
		return -LimelightHelpers.getCameraPose3d_TargetSpace(LimelightConstants.llTags).getZ();
	}



	

	@Override
	public void periodic() { 
		// This method will be called once per scheduler run
	}
}
