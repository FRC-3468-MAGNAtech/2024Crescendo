// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class SetPoseCmd extends Command {


	private final Pose2d pose;

	public SetPoseCmd(Pose2d pose) {
		this.pose = pose;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		SwerveSys.setPose(pose);
		SwerveSys.setHeading(pose.getRotation());
	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {

		return true;

	}
	
}
