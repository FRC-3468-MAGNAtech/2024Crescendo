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

		angleMap.put(0.858, 0.3748);
		angleMap.put(1.978, 0.3806);
		angleMap.put(2.086, 0.4350);
		angleMap.put(2.460, 0.440);
		angleMap.put(2.100, 0.430);
		angleMap.put(3.100, 0.4550);
		angleMap.put(3.800, 0.4550);
		angleMap.put(4.900, 0.4680);
		angleMap.put(2.300, 0.4392);
		angleMap.put(1.730, 0.4365);
		angleMap.put(0.770, 0.3926);
		angleMap.put(2.530, 0.440);
		angleMap.put(2.800, 0.460);
		angleMap.put(4.190, 0.4726);
		angleMap.put(4.350, 0.4735);
		angleMap.put(3.790, 0.4726);
		angleMap.put(2.31, 0.4492);
		angleMap.put(2.27, 0.4453);

	}
	
	public static double getTZ() {
		return -LimelightHelpers.getCameraPose3d_TargetSpace(LimelightConstants.llTags).getZ();
	}



	

	@Override
	public void periodic() { 
		// This method will be called once per scheduler run
	}
}
