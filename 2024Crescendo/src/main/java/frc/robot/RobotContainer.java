// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.Climb;
import frc.robot.commands.Arm.ArmLower;
import frc.robot.commands.Arm.ArmRaise;
import frc.robot.commands.Arm.ArmStop;
import frc.robot.commands.Climb.ClimbDown;
import frc.robot.commands.Climb.ClimbHome;
import frc.robot.commands.Climb.ClimbUp;
import frc.robot.commands.Intake.ExtakeRing;
import frc.robot.commands.Intake.IntakeRing;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Shooter.AmpShooter;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.AutoCommands.GetNoteAuto;
import frc.robot.commands.drivetrain.SwerveDrive;
//import frc.robot.commands.routines.TestRoutine;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {

	// Initialize subsystems.
	public final static SwerveSys swerveSys = new SwerveSys();
	public final static LEDs led = new LEDs();

	// Initialize joysticks.
	private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
	private final JoystickButton zeroGyro = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
	private final JoystickButton turtleEnable = new JoystickButton(driverController.getHID(), XboxController.Button.kBack.value);
	private final JoystickButton aButton = new JoystickButton(driverController.getHID(), XboxController.Button.kA.value);
	private final JoystickButton bButton = new JoystickButton(driverController.getHID(), XboxController.Button.kB.value);

	// Initialize secondary controller stuff
	private final XboxController secondaryDriveController = new XboxController(driveControllerConstants.secondaryDriveControllerPort);
	private final Shooter m_shooter = new Shooter();
	private final Intake m_intake = new Intake();
	private final Arm m_arm = new Arm();
	private final Climb m_climb = new Climb();

	// Initialize auto selector.
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configDriverBindings();

		if (isRedAlliance())
			led.makeItRed();
		else
			led.makeItBlue();

		LimelightConstants.llPIDctrlRotate.setTolerance(0.5);
		LimelightConstants.llPIDctrlDrive.setSetpoint(45);
		LimelightConstants.llPIDctrlDrive.setTolerance(3);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto", autoChooser);

		autoChooser.addOption("BaseAuto", new GetNoteAuto());
		m_climb.setDefaultCommand(new ClimbHome(m_climb));

		m_intake.setDefaultCommand(new IntakeStop(m_intake));
		m_shooter.setDefaultCommand(new ShooterStop(m_shooter));
		m_arm.setDefaultCommand(new ArmStop(m_arm));
	}

	public static boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent())
			return alliance.get() == DriverStation.Alliance.Red;
		return false;
	}

	public void configDriverBindings() {
		swerveSys.setDefaultCommand(new SwerveDrive(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
			() -> aButton.getAsBoolean(),
			() -> bButton.getAsBoolean(),
			true,
			true,
			swerveSys
		));

		// Turtle and Gyro buttons
		turtleEnable.onTrue(new InstantCommand(() -> swerveSys.setTurtleMode()));
		zeroGyro.onTrue(new InstantCommand(() -> swerveSys.resetHeading()));

		//aButton.whileTrue(new RotateToTarget(swerveSys));

		driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
		.whileTrue(Commands.runOnce(() -> swerveSys.lock()));

		configureSecondBindings();
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
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
	private void configureSecondBindings() {

		// JoystickButton m_ArmUpButton = new JoystickButton(
		// secondaryDriveController, 
		// OperatorConstants.ClimbAscendButton);

		// JoystickButton m_ArmDownButton = new JoystickButton(
		// secondaryDriveController, 
		// OperatorConstants.ClimbDescendButton);

		// JoystickButton m_ArmHomeButton = new JoystickButton(
		// secondaryDriveController, 
		// OperatorConstants.ClimbHomeButton);

		// JoystickButton Speaker = new JoystickButton(
		// secondaryDriveController,
		// driveControllerConstants.speakerShooterButton); 

		// JoystickButton Amp = new JoystickButton(
		// secondaryDriveController, 
		// driveControllerConstants.ampShooterButton);

		// JoystickButton intakeButton = new JoystickButton(
		// secondaryDriveController,
		// driveControllerConstants.intakeButton);

		// JoystickButton raiseButton = new JoystickButton(
		// secondaryDriveController, 
		// driveControllerConstants.armRaiseButton);

		// JoystickButton lowerButton = new JoystickButton(
		// secondaryDriveController,
		// driveControllerConstants.armLowerButton);


		// // buttons
		// m_ArmDownButton.whileTrue(new ClimbDown(m_climb));
		// m_ArmUpButton.whileTrue(new ClimbUp(m_climb));
		// m_ArmHomeButton.onTrue(new ClimbHome(m_climb));
		// Speaker.onTrue(new Shoot(m_shooter));
		// Amp.onTrue(new AmpShooter(m_shooter));
		// intakeButton.whileTrue(new IntakeRing(m_intake));
		// raiseButton.whileTrue(new ArmRaise(m_arm));
		// lowerButton.whileTrue(new ArmLower(m_arm));

		JoystickButton intake = new JoystickButton(secondaryDriveController, SecondDriveControllerConstants.intakeButton);
		JoystickButton shoot = new JoystickButton(secondaryDriveController, SecondDriveControllerConstants.shootButton);
		JoystickButton armUp = new JoystickButton(secondaryDriveController, SecondDriveControllerConstants.shootArmRaiseButton);
		JoystickButton armDown = new JoystickButton(secondaryDriveController, SecondDriveControllerConstants.shootArmLowerButton);
		JoystickButton climbUp = new JoystickButton(secondaryDriveController, SecondDriveControllerConstants.climbDownButton);
		JoystickButton climbDown = new JoystickButton(secondaryDriveController, SecondDriveControllerConstants.climbUpBotton);
		JoystickButton extake = new JoystickButton(secondaryDriveController, SecondDriveControllerConstants.extakeButton);

		intake.whileTrue(new IntakeRing(m_intake));
		extake.whileTrue(new ExtakeRing(m_intake));
		shoot.whileTrue(new Shoot(m_shooter));
		armUp.whileTrue(new ArmRaise(m_arm));
		armDown.whileTrue(new ArmLower(m_arm));
		climbUp.whileTrue(new ClimbUp(m_climb));
		climbDown.whileTrue(new ClimbDown(m_climb));
	}

	// For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
	public void updateInterface() {
		SmartDashboard.putNumber("heading degrees", SwerveSys.getHeading().getDegrees());
		SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSec());

		SmartDashboard.putNumber("FR angle degrees", swerveSys.getModuleStates()[0].angle.getDegrees());
		SmartDashboard.putNumber("FL angle degrees", swerveSys.getModuleStates()[1].angle.getDegrees());
		SmartDashboard.putNumber("BR angle degrees", swerveSys.getModuleStates()[2].angle.getDegrees());
		SmartDashboard.putNumber("BL angle degrees", swerveSys.getModuleStates()[3].angle.getDegrees());

		SmartDashboard.putNumber("FR CANCoder", SwerveSys.frontLeftMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);
		SmartDashboard.putNumber("FL CANCoder", SwerveSys.frontRightMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);
		SmartDashboard.putNumber("BR CANCoder", SwerveSys.backLeftMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);
		SmartDashboard.putNumber("BL CANCoder", SwerveSys.backRightMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);

		SmartDashboard.putNumber("Limelight TA", LimelightHelpers.getTA("limelight-notes"));
		SmartDashboard.putNumber("Limelight TX", LimelightHelpers.getTX("limelight-notes"));
		SmartDashboard.putNumber("Limelight TY", LimelightHelpers.getTY("limelight-notes"));
	}
}
