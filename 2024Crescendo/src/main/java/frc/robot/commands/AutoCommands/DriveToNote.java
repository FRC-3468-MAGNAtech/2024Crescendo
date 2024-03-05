// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSys;

public class DriveToNote extends Command {
	SwerveSys swerveSys;
	/** Creates a new DriveToNote. */
	public DriveToNote(SwerveSys sys) {
		// Use addRequirements() here to declare subsystem dependencies.
		swerveSys = sys;
		addRequirements(swerveSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double rot = Targeting.aimToNote();
		double drive = Targeting.driveToNote();

		/**
		 * Square inputs
		 */
		double theta = Math.atan2(drive, 0);
		double r = Math.pow(Math.hypot(drive, 0), 2.0);

		drive = r * Math.sin(theta);

		rot = Math.copySign(Math.pow(rot, 2.0), rot);

		//robot go go
		swerveSys.drive(
			-drive,
			0,
			-rot,
			false
		);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return LimelightConstants.llPIDctrlDrive.atSetpoint();
	}
}
