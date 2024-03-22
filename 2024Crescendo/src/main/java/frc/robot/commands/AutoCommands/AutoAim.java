// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Targeting;
import frc.robot.subsystems.*;

public class AutoAim extends Command {
	SwerveSys m_swerve;
	Arm m_arm;
	double armAngle = 0;
	/** Creates a new AutoAim. */
	public AutoAim(SwerveSys swerve, Arm arm) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(swerve, arm);
		m_swerve = swerve;
		m_arm = arm;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double rot = 0;
		if (!LimelightConstants.llPIDctrlRotate.atSetpoint()) {
			LimelightHelpers.setPipelineIndex(LimelightConstants.llTags, 1);
			rot = Targeting.aimToAprilTag();
		}
		else
        	LimelightHelpers.setPipelineIndex(LimelightConstants.llTags, 0);
		armAngle = Targeting.aimArmToSpeaker();
		m_swerve.drive(0, 0, rot, false);
		m_arm.pointMove(armAngle);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_swerve.stop();
		m_arm.stop();
		System.out.println("Aimed");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		double angle = m_arm.getAngle();
		return angle > armAngle - 0.1 && angle < armAngle + 0.1;
	}
}
