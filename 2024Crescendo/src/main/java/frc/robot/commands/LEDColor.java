// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;

public class LEDColor extends Command {
	LEDs m_led;
	/** Creates a new LEDAllianceColor. */
	public LEDColor(LEDs led) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_led = led;
		addRequirements(led);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_led.setShooterAllianceColor();
		if (RobotContainer.m_intake.getIntakeSensor()) {
			m_led.makeItGreen();
			return;
		}
		if (RobotContainer.isRedAlliance())
			m_led.makeItRed();
		else
			m_led.makeItBlue();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
