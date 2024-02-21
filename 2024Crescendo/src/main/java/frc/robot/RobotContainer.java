package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.drivetrain.SwerveDrive;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {

    
    // Initialize subsystems.
    public  final static SwerveSys swerveSys = new SwerveSys();
    private final SendableChooser<Command> autoChooser;

    // Initialize joysticks.
    private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
    private final JoystickButton zeroGyro = new JoystickButton(driverController.getHID(), XboxController.Button.kBack.value);

    // Initialize auto selector.

    public RobotContainer() {
        configDriverBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);
        autoChooser.addOption("TestScoring", new PathPlannerAuto("TestScoring"));
        autoChooser.addOption("Straight", new PathPlannerAuto("straight Auto"));
    }

    public void configDriverBindings() {
        swerveSys.setDefaultCommand(new SwerveDrive(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            () -> zeroGyro.getAsBoolean(),
            true,
            true,
            swerveSys
        ));

        // FIXME: Consider building simple commands this way instead of creating a whole file for them.
        // If you're more comfortable with it, you still can use the other way (i.e. new ResetHeadingCmd(swerveSys)).
        // Otherwise I would delete those simple commands just to keep things clean.

        // Start is the "three lines" button. Back is the "windows" button.
        driverController.start().onTrue(Commands.runOnce(() -> swerveSys.resetHeading()));

        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .whileTrue(Commands.runOnce(() -> swerveSys.lock()));

    }

    public Command getAutonomousCommand() {
        // swerveSys.setHeading(Rotation2d.fromDegrees(73));
        // swerveSys.resetHeading();

        return autoChooser.getSelected();
    }

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    public void updateInterface() {
        SmartDashboard.putNumber("heading degrees", swerveSys.getHeading().getDegrees());
        SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSec());

        SmartDashboard.putNumber("FR angle degrees", swerveSys.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("FL angle degrees", swerveSys.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("BR angle degrees", swerveSys.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("BL angle degrees", swerveSys.getModuleStates()[3].angle.getDegrees());

        SmartDashboard.putNumber("FR CANCoder", swerveSys.frontLeftMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("FL CANCoder", swerveSys.frontRightMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("BR CANCoder", swerveSys.backLeftMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("BL CANCoder", swerveSys.backRightMod.canCoder.getAbsolutePosition().getValueAsDouble() * 360);

        SmartDashboard.putNumber("Pigeon Yaw", swerveSys.imu.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("PigeonX", swerveSys.imu.getAccumGyroX().getValueAsDouble());
        SmartDashboard.putNumber("PigeonY", swerveSys.imu.getAccumGyroY().getValueAsDouble());
    }
}
