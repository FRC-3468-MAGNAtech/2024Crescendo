// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbDown extends Command {
  /** Creates a new ClimbDown. */
  private Climb i_subsystem;
  /** Creates a new LeftArmDescendSpeed. */
  public ClimbDown(Climb subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    i_subsystem = subsystem;

    addRequirements(subsystem);
  }

  public ClimbDown() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i_subsystem.setLeftDescendSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
