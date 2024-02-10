// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AmpOoze extends Command {
   Shooter ooze;
  /** Creates a new ShooterCommands. 
   * @return */
  public void amp(Shooter subsytem) {
    
      ooze = subsytem;
      addRequirements(ooze);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ooze.amp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ooze.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
