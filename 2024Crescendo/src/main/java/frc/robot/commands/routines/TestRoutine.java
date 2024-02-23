// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routines;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.GetPoseCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestRoutine extends SequentialCommandGroup {
  /** Creates a new TestRoutine. */
  public TestRoutine() {

    Pose2d targetPose = new Pose2d(7.2, 6.6, Rotation2d.fromDegrees(180));
    PathConstraints constraints = new PathConstraints(3, 2, Units.degreesToRadians(540), Units.degreesToRadians(720));
    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints,0,0);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GetPoseCamera(),
      pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints,0,0)
    );
  }
}
