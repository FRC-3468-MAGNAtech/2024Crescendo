// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class SetTranslationCmd extends Command {

	//private final SwerveSys swerveSys;

	//private final Translation2d translation;

	public SetTranslationCmd(Translation2d translation, SwerveSys swerveSys) {
		//this.swerveSys = swerveSys;
		//this.translation = translation;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		//swerveSys.setTranslation(translation);
	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return true;
	}
	
}
