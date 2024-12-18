// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import java.lang.annotation.Retention;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command {
	private Climb i_subsystem;
	
	/** Creates a new ClimbUp. */
	public ClimbUp(Climb subsystem) {
		i_subsystem = subsystem;
		addRequirements(subsystem);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (i_subsystem.getEncoderValue() >= ClimbConstants.upPIDReference) {
			i_subsystem.stopArms();
			return;
		}
		i_subsystem.setLeftAscendSpeed();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		i_subsystem.stopArms();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
