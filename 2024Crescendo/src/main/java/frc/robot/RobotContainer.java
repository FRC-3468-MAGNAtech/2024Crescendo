// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.AmpShooter;
import frc.robot.commands.ArmLower;
import frc.robot.commands.ArmRaise;
import frc.robot.commands.ArmStop;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake.IntakeRing;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Shooter.AmpOoze;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.StopShootAmp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize subsystems/controllers
  private final XboxController secondaryDriveController = new XboxController(driveControllerConstants.secondaryDriveControllerPort);
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Arm m_arm = new Arm();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  // // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_intake.setDefaultCommand(new IntakeStop(m_intake));
    // m_shooter.setDefaultCommand(new StopShootAmp(m_shooter));
    m_arm.setDefaultCommand(new ArmStop(m_arm));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Speaker button
    JoystickButton Speaker = new JoystickButton(
      secondaryDriveController,
      driveControllerConstants.speakerShooterButton); 
      
    JoystickButton Amp = new JoystickButton(
      secondaryDriveController, 
      driveControllerConstants.ampShooterButton);

    JoystickButton intakeButton = new JoystickButton(
      secondaryDriveController,
      driveControllerConstants.intakeButton );

    JoystickButton raiseButton = new JoystickButton(
      secondaryDriveController, 
      driveControllerConstants.armRaiseButton);

    JoystickButton lowerButton = new JoystickButton(
      secondaryDriveController,
      driveControllerConstants.armLowerButton);

    


    // buttons
    Speaker.onTrue(new Shoot(m_shooter));
    Amp.onTrue(new AmpShooter(m_shooter));
    intakeButton.whileTrue(new IntakeRing(m_intake));
    raiseButton.whileTrue(new ArmRaise(m_arm));
    lowerButton.whileTrue(new ArmLower(m_arm));
    }
   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
