// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
	private Shooter pew;
	
	/** Creates a new Shoot. */
	public Shoot(Shooter subsytem) {
		pew = subsytem;
		addRequirements(subsytem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		pew.shoot();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		pew.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (pew.getVelocity() < -3600.0){
			return true;
		}
		return false;
	}
}
