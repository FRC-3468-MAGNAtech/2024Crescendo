// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.subsystems.SwerveSys;


/** Add your docs here. */
public class PPTrajectory {
    public static SwerveSys Swerve = RobotContainer.swerveSys;
    public static PPTrajectory INSTANCE = new PPTrajectory();
    public static HashMap<String, Command> eventMap;
    public static ReplanningConfig config  = new ReplanningConfig(true, true);
    public static HolonomicPathFollowerConfig swerveFollowerConfig = new HolonomicPathFollowerConfig(Constants.DriveConstants.maxDriveSpeedMetersPerSec, Constants.DriveConstants.driveBaseRadius, config);

    public static PathPlannerPath getTestPath() {
        return PathPlannerPath.fromPathFile("TestPath");
    }

    public PPTrajectory() {
        eventMap = new HashMap<>();

        AutoBuilder.configureHolonomic(() -> Swerve.getPose(), (Pose2d pose) -> Swerve.resetPose(), () -> Swerve.getChassisSpeeds(), (ChassisSpeeds speeds) -> Swerve.setChassisSpeeds(speeds), swerveFollowerConfig, () -> honk(), Swerve);
    }

    private boolean honk() {
        return false;
    }


}
