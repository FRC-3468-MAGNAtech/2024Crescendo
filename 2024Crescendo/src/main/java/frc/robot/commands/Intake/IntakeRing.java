// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeRing extends Command {
	private Intake i_subsystem;

	/** Creates a new IntakeRing. */
	public IntakeRing(Intake subsystem) {
		i_subsystem = subsystem;
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		i_subsystem.intake();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		i_subsystem.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return i_subsystem.getIntakeSensor();
	}
}
