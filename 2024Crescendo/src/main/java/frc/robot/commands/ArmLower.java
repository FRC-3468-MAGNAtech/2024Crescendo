// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmLower extends Command {
  /** Creates a new ArmLower. */

  private Arm i_subsystem;

  public ArmLower( Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = i_subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i_subsystem.lower();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    i_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
