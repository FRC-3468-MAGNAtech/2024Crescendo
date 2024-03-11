// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Targeting;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSys;

public class SwerveDrive extends Command {

	/**
	 * Command to allow for driver input in teleop
	 * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
	 */
	private final SwerveSys swerveSys;

	/**
	 * Joysticks return DoubleSuppliers when the get methods are called
	 * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
	 * versus using a double which would only update when the constructor is called
	 */
	private final DoubleSupplier drive;
	private final DoubleSupplier strafe;
	private final DoubleSupplier rot;

	private final BooleanSupplier aPressed;

	private final boolean isFieldRelative;
	private final boolean squareInputs;


	/**
	 * Constructs a new ArcadeDriveCmd.
	 * 
	 * <p>ArcadeDriveCmd is used to control the swerve drive base with arcade drive.
	 * 
	 * @param drive The commanded forward/backward lateral motion.
	 * @param strafe The commanded left/right lateral motion.
	 * @param rot The commanded rotational motion.
	 * @param isFieldRelative Whether the commanded inputs are field- or robot-oriented.
	 * @param squareInputs Whether the commanded inputs should be squared or linear.
	 * @param swerveSys The required SwerveSys.
	 */
	public SwerveDrive(
		DoubleSupplier drive, 
		DoubleSupplier strafe, 
		DoubleSupplier rot,
		BooleanSupplier aPressed,
		boolean isFieldRelative,
		boolean squareInputs,
		SwerveSys swerveSys
	) {
		this.swerveSys = swerveSys;

		this.drive = drive;
		this.strafe = strafe;
		this.rot = rot;
		this.aPressed = aPressed;
		this.isFieldRelative = isFieldRelative;
		this.squareInputs = squareInputs;

		addRequirements(swerveSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double drive = this.drive.getAsDouble();
		double strafe = this.strafe.getAsDouble();
		double rot = this.rot.getAsDouble();
		boolean aPressed = this.aPressed.getAsBoolean();
		boolean currentlyFieldRelative = isFieldRelative;

		// If the A button is pressed, make it to where the robot automatically aims toward an AprilTag
		if (aPressed) {
        	LimelightHelpers.setPipelineIndex(LimelightConstants.llTags, 1);
			rot = Targeting.aimToAprilTag();
		}
		else
        	LimelightHelpers.setPipelineIndex(LimelightConstants.llTags, 0);

		if(squareInputs) {
			// Squaring inputs while preserving commanded lateral direction
			double theta = Math.atan2(drive, strafe);
			double r = Math.pow(Math.hypot(drive, strafe), 2.0);

			drive = r * Math.sin(theta);
			strafe = r * Math.cos(theta);

			rot = Math.copySign(Math.pow(rot, 2.0), rot);
		}

		//robot go go
		swerveSys.drive(
			-drive,
			-strafe,
			-rot,
			currentlyFieldRelative
		);
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