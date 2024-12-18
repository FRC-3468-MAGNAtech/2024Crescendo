// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class Shoot2 extends Command {
	private Shooter pew;
	private LEDs led;
	private Timer timer = new Timer();
	
	/** Creates a new Shoot. */
	public Shoot2(Shooter subsytem, LEDs led) {
		pew = subsytem;
		this.led = led;
		addRequirements(subsytem, led);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		led.setColor(LEDs.LEDColor.Magenta);
		timer.reset();
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		pew.shoot();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		pew.stop();
		timer.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.hasElapsed(0.2);
	}
}
