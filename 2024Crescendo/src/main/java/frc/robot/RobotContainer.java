// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.auto.routines.TestPath;
import frc.robot.commands.drivetrain.SwerveDrive;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {
    
    // Initialize subsystems.
    public  final static SwerveSys swerveSys = new SwerveSys();

    // Initialize joysticks.
    private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
    private final JoystickButton zeroGyro = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
    private final JoystickButton turtleEnable = new JoystickButton(driverController.getHID(), XboxController.Button.kBack.value);

    // Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        SmartDashboard.putData("auto selector", autoSelector);

        // Add programs to auto selector.
        autoSelector.setDefaultOption("Do Nothing", null);
        autoSelector.addOption("TestPath", new TestPath(swerveSys));
        
        configDriverBindings();
    }

    public void configDriverBindings() {
        swerveSys.setDefaultCommand(new SwerveDrive(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys
        ));
        
        // Turtle and Gyro buttons
        turtleEnable.onTrue(new InstantCommand(() -> swerveSys.setTurtleMode()));
        zeroGyro.onTrue(new InstantCommand(() -> swerveSys.resetHeading()));

        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .whileTrue(Commands.runOnce(() -> swerveSys.lock()));
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
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

        SmartDashboard.putNumber("Speed Factor", swerveSys.getSpeedFactor());
    }
}
