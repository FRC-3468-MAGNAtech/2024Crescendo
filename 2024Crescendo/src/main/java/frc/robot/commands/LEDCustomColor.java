// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDColor;

public class LEDCustomColor extends Command {
	LEDs m_led;
	LEDColor color = LEDColor.Black;
	/** Creates a new LEDAllianceColor. */
	public LEDCustomColor(LEDs led, LEDColor color) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_led = led;
		this.color = color;
		addRequirements(led);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		switch (color) {
			case Black:
				m_led.clearColor();
				break;
			case White:
				m_led.makeItWhite();
				break;
			case Red:
				m_led.makeItRed();
				break;
			case Green:
				m_led.makeItGreen();
				break;
			case Blue:
				m_led.makeItBlue();
				break;
			case Magenta:
				m_led.makeItMagenta();
				break;
			case Yellow:
				m_led.makeItYellow();
				break;
			case Cyan:
				m_led.makeItCyan();
				break;
		}
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
