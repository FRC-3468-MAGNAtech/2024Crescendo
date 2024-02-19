// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

/** Add your docs here. */
public class PathPlanner {


    public final static SendableChooser<Command> autoChooser=AutoBuilder.buildAutoChooser("New Auto");

    public static void SetPathPlannerSettings() {
        setSmartDashboard();
        FindPath();
        addAutoOptions();
    }
    public static void setSmartDashboard() {

        SmartDashboard.putData("Auto", autoChooser);
    }
    public static void FindPath(){

        autoChooser.addOption("TestPath", AutoBuilder.pathfindToPose(
        new Pose2d(5.87,1.45, Rotation2d.fromDegrees(0)),
        new PathConstraints(3.25, 1,
         Units.degreesToRadians(540), Units.degreesToRadians(720)),
         1,0));
     
    }
    public static void addAutoOptions() {
        autoChooser.addOption("TestPath", Commands.runOnce(() -> followPathCommand("TestPath")));

        try (Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner","paths"))) {//deploy/patplanner/paths klasörünün içindekilerinin alınması
      files.filter(file -> !Files.isDirectory(file))//sadece dosyaların alınması
          .map(Path::getFileName)//dosya adlarının alınması
          .map(Path::toString)//dosya adlarının stringe çevrilmesi
          .filter(fileName -> fileName.endsWith(".path"))// Sadece ".path" uzantılı dosyaların seçilmesi.
          .sorted()//seçilernlerin sıralanması
          .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))//sadece noktadan önceki kısmı alır
          .forEach(pathName -> autoChooser.addOption("PP:" + pathName,Commands.runOnce(() -> {followPathCommand(pathName);})));
    } catch (IOException e) {
      System.out.println("********* Failed to list PathPlanner paths. *********");
    }
    }
    public static Command followPathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        HolonomicPathFollowerConfig conf = new HolonomicPathFollowerConfig(
            Constants.AutoConstants.driveConstants,
            Constants.AutoConstants.rotConstants, 
            DriveConstants.maxDriveSpeedMetersPerSec,
            DriveConstants.wheelBase/2,
            new ReplanningConfig(true, false)
            );
        return new FollowPathHolonomic(
            path, 
            RobotContainer.swerveSys::getPose, 
            RobotContainer.swerveSys::getChassisSpeeds,
            RobotContainer.swerveSys::setChassisSpeeds,
            conf,
            RobotContainer.swerveSys::PathFlip, 
            RobotContainer.swerveSys);
    }


}
