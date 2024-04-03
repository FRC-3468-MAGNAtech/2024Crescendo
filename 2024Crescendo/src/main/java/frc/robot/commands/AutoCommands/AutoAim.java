// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.subsystems.*;

public class AutoAim extends Command {
	Arm m_arm;
	double armAngle = 0;
	/** Creates a new AutoAim. */
	public AutoAim(Arm arm) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(arm);
		m_arm = arm;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		armAngle = Targeting.aimArmToSpeakerInt();
		m_arm.pointMove(armAngle);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_arm.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		double angle = m_arm.getAngle();
		return angle > armAngle - 0.01 && angle < armAngle + 0.01;
	}
}
