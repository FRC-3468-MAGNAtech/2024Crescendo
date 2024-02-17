// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import java.security.PublicKey;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.PPTrajectory;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.subsystems.SwerveSys;

/** Add your docs here. */
public class TestPath extends SequentialCommandGroup {

    public TestPath(SwerveSys swerveSys) {
        Command testPathCommand = AutoBuilder.buildAuto("TestPath");

        addCommands(
            new ResetPoseCmd(swerveSys),
            testPathCommand
        );
    }

}
