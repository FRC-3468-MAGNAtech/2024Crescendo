// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmRaise extends Command {
  /** Creates a new ArmRaise. */
    Arm uppies;
  /** Creates a new ShooterCommands. 
   * @return */
  public void raise(Arm subsytem) {
    
      uppies = subsytem;
      addRequirements(uppies);
  }
  public ArmRaise(Arm m_armRaise) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    uppies.raise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    uppies.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
