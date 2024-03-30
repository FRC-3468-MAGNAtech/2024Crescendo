// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDColor;
import frc.robot.commands.LEDAllianceColor;
import frc.robot.commands.LEDCustomColor;
import frc.robot.commands.Arm.Amp;
import frc.robot.commands.Arm.ArmLower;
import frc.robot.commands.Arm.ArmRaise;
import frc.robot.commands.Arm.PointMove;
import frc.robot.commands.Arm.PointMoveAuto;
import frc.robot.commands.Arm.Trap;
import frc.robot.commands.Climb.ClimbDown;
import frc.robot.commands.Climb.ClimbUp;
import frc.robot.commands.Intake.ExtakeRing;
import frc.robot.commands.Intake.IntakeRing;
import frc.robot.commands.Intake.IntakeRingS;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Shooter.AmpOoze;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.drivetrain.SwerveDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

	// Initialize subsystems.
	public final static Arm m_arm = new Arm();
	public final static LEDs m_led = new LEDs();
	public final static Shooter m_shooter = new Shooter();
	public final static Intake m_intake = new Intake();
	public final static Climb m_climb = new Climb();
	public final static SwerveSys m_swerveSys = new SwerveSys();
	public final static Camera m_camera = new Camera();

	public static double currentAngle = 0.5;

	// Initialize joysticks.
	private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
	private final JoystickButton zeroGyro = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
	private final JoystickButton turtleEnable = new JoystickButton(driverController.getHID(), XboxController.Button.kBack.value);

	// Initialize secondary controller stuff
	private final XboxController secondaryDriveController = new XboxController(SubsystemControllerConstants.subsystemControllerPort);
	private final JoystickButton amp = new JoystickButton(secondaryDriveController, XboxController.Button.kStart.value);
	private final JoystickButton trap = new JoystickButton(secondaryDriveController, XboxController.Button.kBack.value);
	private final JoystickButton autoAim = new JoystickButton(secondaryDriveController, SubsystemControllerConstants.autoAimButton);

	// Initialize auto selector.
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		RobotContainer.registerNamedCommands();
		m_swerveSys.BuilderConfigure();

		LimelightConstants.llPIDctrlRotate.setTolerance(0.5);
		LimelightConstants.llPIDctrlDrive.setSetpoint(45);
		LimelightConstants.llPIDctrlDrive.setTolerance(3);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto", autoChooser);

		autoChooser.addOption("BaseAuto", new GetNoteAuto());
		autoChooser.addOption("Test3Note", new PathPlannerAuto("TestAuto"));
		autoChooser.addOption("Test2Note", new PathPlannerAuto("Test Auto 2"));
		autoChooser.addOption("CheeseAuto", new PathPlannerAuto("CheeseAuto"));

		m_intake.setDefaultCommand(new IntakeStop(m_intake));
		m_shooter.setDefaultCommand(new ShooterStop(m_shooter));
		m_arm.setDefaultCommand(new PointMove(m_arm));
		m_led.setDefaultCommand(new LEDAllianceColor(m_led));

		LimelightHelpers.setPipelineIndex(LimelightConstants.llTags, 0);
		configDriverBindings();

	}

	public static boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent())
			return alliance.get() == DriverStation.Alliance.Red;
		return false;
	}

	public void configDriverBindings() {
		m_swerveSys.setDefaultCommand(new SwerveDrive(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
			() -> { return false; },
			true,
			true,
			m_swerveSys
		));

		
		JoystickButton intakeSource = new JoystickButton(driverController.getHID(), SubsystemControllerConstants.intakeButton);
		JoystickButton parkButton = new JoystickButton(driverController.getHID(), XboxController.Button.kB.value);

		// Turtle and Gyro buttons
		turtleEnable.onTrue(new InstantCommand(() -> m_swerveSys.setTurtleMode()));
		zeroGyro.onTrue(new InstantCommand(() -> m_swerveSys.resetHeading()));
		driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
		.whileTrue(Commands.runOnce(() -> m_swerveSys.lock()));

		intakeSource.onTrue(new ParallelCommandGroup(
			new InstantCommand(() -> {currentAngle = 0.529;}),
			new PointMove(m_arm)
			)).whileTrue(new IntakeRing(m_intake));

		parkButton.onTrue(new ParallelCommandGroup(
			new InstantCommand(() -> {currentAngle = 0.37;}),
			new PointMove(m_arm)
			));

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

		JoystickButton intake = new JoystickButton(secondaryDriveController, SubsystemControllerConstants.intakeButton);
		JoystickButton shoot = new JoystickButton(secondaryDriveController, SubsystemControllerConstants.shootButton);
		JoystickButton armUp = new JoystickButton(secondaryDriveController, SubsystemControllerConstants.shootArmRaiseButton);
		JoystickButton armDown = new JoystickButton(secondaryDriveController, SubsystemControllerConstants.shootArmLowerButton);
		JoystickButton extake = new JoystickButton(secondaryDriveController, SubsystemControllerConstants.extakeButton);

		Trigger climbUp = new Trigger(() -> { return secondaryDriveController.getRightTriggerAxis() > 0.5; });
		Trigger climbDown = new Trigger(() -> { return secondaryDriveController.getLeftTriggerAxis() > 0.5; });
		Trigger hasNote = new Trigger(() -> { return m_intake.getIntakeSensor(); });

		// intake.whileTrue(new IntakeRing(m_intake));
		/*autoAim.onTrue(new SequentialCommandGroup(
			new InstantCommand(() -> {currentAngle = Targeting.aimArmToSpeakerInt();}),
			new PointMove(m_arm)
		));*/

		autoAim.onTrue(new SequentialCommandGroup(
			new AutoAim(m_arm),
			new SequentialCommandGroup(new Shoot(m_shooter, m_led), 
			new ParallelCommandGroup(new Shoot(m_shooter, m_led), new IntakeRingS(m_intake)))
		));

		intake.onTrue(new ParallelCommandGroup(
			new InstantCommand(() -> {currentAngle = armConstants.intakeSetPoint;}),
			new PointMove(m_arm)
			)).whileTrue(new IntakeRing(m_intake));

		extake.whileTrue(new ExtakeRing(m_intake));
		shoot.whileTrue(new SequentialCommandGroup( new Shoot(m_shooter, m_led), 
			new ParallelCommandGroup(new Shoot(m_shooter, m_led), new IntakeRingS(m_intake))));

		armUp.whileTrue(new ArmRaise(m_arm));
		armDown.whileTrue(new ArmLower(m_arm));

		hasNote.whileTrue(new LEDCustomColor(m_led, LEDColor.Green));

		climbUp.whileTrue(new ClimbUp(m_climb));
		climbDown.whileTrue(new ClimbDown(m_climb));
		amp.whileTrue(new SequentialCommandGroup( new Amp(m_arm), new ParallelCommandGroup( new AmpOoze(m_shooter), new IntakeRing(m_intake) )));
		trap.whileTrue(
			new SequentialCommandGroup(
				new Trap(m_arm), 
				new SequentialCommandGroup( 
					new Shoot(m_shooter, m_led), 
					new ParallelCommandGroup(
						new Trap(m_arm), 
						new Shoot(m_shooter, m_led), 
						new IntakeRing(m_intake)


					)
				)
			)
		);
	}

	public static void registerNamedCommands() {
		NamedCommands.registerCommand("Intake", new IntakeRing(m_intake));
		NamedCommands.registerCommand("AutoAim", new AutoAim(m_arm));
		NamedCommands.registerCommand("KeepArmUp", new PointMove(m_arm));
		NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(new Shoot(m_shooter, m_led), 
			new ParallelCommandGroup(new Shoot(m_shooter, m_led), new IntakeRingS(m_intake))));
		NamedCommands.registerCommand("SimplePark", new ParallelCommandGroup(
			new InstantCommand(() -> {currentAngle = 0.35;}),
			new PointMoveAuto(m_arm)));
		NamedCommands.registerCommand("ZeroGyro", new InstantCommand(() -> m_swerveSys.resetHeading()));
		
	}

	// For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
	public void updateInterface() {
		SmartDashboard.putNumber("ArmSetAngle", currentAngle);
		SmartDashboard.putNumber("speed m/s", m_swerveSys.getAverageDriveVelocityMetersPerSec());
		SmartDashboard.putNumber("DistanceFromSpeaker", Camera.getTZ());
	}
}
