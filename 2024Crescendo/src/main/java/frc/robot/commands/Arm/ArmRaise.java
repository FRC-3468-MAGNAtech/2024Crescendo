// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmRaise extends Command {
	private Arm uppies;
	
	/** Creates a new ArmRaise. */
	public ArmRaise(Arm subsytem) {
		uppies = subsytem;
		addRequirements(uppies);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		uppies.raise();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		uppies.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (uppies.getAngle() > 0.62) {
			uppies.stop();
			return true;
		}
		return false;
	}
}
