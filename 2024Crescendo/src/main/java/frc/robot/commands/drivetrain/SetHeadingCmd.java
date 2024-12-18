// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class SetHeadingCmd extends Command {


	private final Rotation2d heading;

	public SetHeadingCmd(Rotation2d heading) {
		this.heading = heading;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		SwerveSys.setHeading(heading);
	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {

		return true;

	}
	
}
