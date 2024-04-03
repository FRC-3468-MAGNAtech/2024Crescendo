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

		angleMap.put(0.770, 0.3926);
		angleMap.put(0.858, 0.3948);
		angleMap.put(1.120, 0.4300);
		angleMap.put(1.490, 0.3900);
		angleMap.put(1.730, 0.4365);
		angleMap.put(1.978, 0.3906);
		angleMap.put(2.086, 0.4450);
		angleMap.put(2.100, 0.4464);
		angleMap.put(2.160, 0.4474);
		angleMap.put(2.200, 0.4476);
		angleMap.put(2.230, 0.4476);
		angleMap.put(2.270, 0.4500);
		angleMap.put(2.300, 0.4500);
		angleMap.put(2.310, 0.4590);
		angleMap.put(2.430, 0.4600);
		angleMap.put(2.460, 0.4610);
		angleMap.put(2.530, 0.4620);
		angleMap.put(2.750, 0.4630);
		angleMap.put(2.800, 0.4640);
		angleMap.put(3.100, 0.4650);
		angleMap.put(3.520, 0.4660);
		angleMap.put(3.790, 0.4670);
		angleMap.put(3.800, 0.4680);
		angleMap.put(4.190, 0.4690);
		angleMap.put(4.350, 0.4700);
		angleMap.put(4.900, 0.4700);

	}
	
	public static double getTZ() {
		return -LimelightHelpers.getCameraPose3d_TargetSpace(LimelightConstants.llTags).getZ();
	}



	

	@Override
	public void periodic() { 
		// This method will be called once per scheduler run
	}
}
