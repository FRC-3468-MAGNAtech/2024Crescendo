// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class SwerveSys extends SubsystemBase {

    // Initializes swerve module objects
    public final static SwerveModule frontLeftMod = 
        new SwerveModule(
            CANDevices.frontLeftDriveMtrId,
            CANDevices.frontLeftSteerMtrId,
            CANDevices.frontLeftCanCoderId,
            DriveConstants.frontLeftModOffset,
            DriveConstants.frontLeftDriveInvert,
            DriveConstants.frontLeftSteerInvert
        );

    public final static SwerveModule frontRightMod = 
        new SwerveModule(
            CANDevices.frontRightDriveMtrId,
            CANDevices.frontRightSteerMtrId,
            CANDevices.frontRightCanCoderId,
            DriveConstants.frontRightModOffset,
            DriveConstants.frontRightDriveInvert,
            DriveConstants.frontRightSteerInvert
        );

    public final static SwerveModule backLeftMod = 
        new SwerveModule(
            CANDevices.backLeftDriveMtrId,
            CANDevices.backLeftSteerMtrId,
            CANDevices.backLeftCanCoderId,
            DriveConstants.backLeftModOffset,
            DriveConstants.backLeftDriveInvert,
            DriveConstants.backLeftSteerInvert
        );

    public final static SwerveModule backRightMod = 
        new SwerveModule(
            CANDevices.backRightDriveMtrId,
            CANDevices.backRightSteerMtrId,
            CANDevices.backRightCanCoderId,
            DriveConstants.backRightModOffset,
            DriveConstants.backRightDriveInvert,
            DriveConstants.backRightSteerInvert
        );

    private boolean isLocked = false;
    public boolean isLocked() {
        return isLocked;
    }

    private boolean isFieldOriented = true;
    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    private double speedFactor = 1.0;
    public double getSpeedFactor() {
        return speedFactor;
    }
    /**
     * Sets the speed factor of the robot. Inputs are multiplied by this factor to reduce drive speed.
     * Useful for "turtle" or "sprint" modes.
     * @param speedFactor The factor to scale inputs, as a percentage.
     */
    public void setSpeedFactor(double speedFactor) {
        this.speedFactor = speedFactor;
    }

    public final static Pigeon2 imu = new Pigeon2(CANDevices.imuId);

    // Odometry for the robot, measured in meters for linear motion and radians for rotational motion
    // Takes in kinematics and robot angle for parameters

    private static SwerveDrivePoseEstimator odometry = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d()
        );

    /**
     * Constructs a new SwerveSys.
     * 
     * <p>SwerveCmd contains 4 {@link SwerveModule}, a gyro, and methods to control the drive base and odometry.
     */
    public SwerveSys() {
        // Resets the measured distance driven for each module
        frontLeftMod.resetDriveDistance();
        frontRightMod.resetDriveDistance();
        backLeftMod.resetDriveDistance();
        backRightMod.resetDriveDistance();

        resetHeading();
        resetPose(new Pose2d());

        
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Updates the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());
    }
    
    /**
     * Inputs drive values into the swerve drive base.
     * 
     * @param driveX The desired forward/backward lateral motion, in meters per second.
     * @param driveY The desired left/right lateral motion, in meters per second.
     * @param rotation The desired rotational motion, in radians per second.
     * @param isFieldOriented whether driving is field- or robot-oriented.
     */
    public void drive(double driveX, double driveY, double rotation, boolean isFieldOriented) {  
        if(driveX != 0.0 || driveY != 0.0 || rotation != 0.0) isLocked = false;
        
        if(isLocked) {
            setModuleStatesOpenLoop(new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
            });
        }
        else {
            // Reduces the speed of the drive base for "turtle" or "sprint" modes.
            driveX *= speedFactor;
            driveY *= speedFactor;
            rotation *= speedFactor;

            // Represents the overall state of the drive base.
            ChassisSpeeds speeds =
            isFieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveX, driveY, rotation, getHeading())
                : new ChassisSpeeds(driveX, driveY, rotation);

            // Uses kinematics (wheel placements) to convert overall robot state to array of individual module states.
            SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
            
            // Makes sure the wheels don't try to spin faster than the maximum speed possible
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeedMetersPerSec);

            setModuleStatesOpenLoop(states);
        }
    }

    /**
     * Stops the driving of the drive base.
     * <p>Sets all drive inputs to zero. This will set the drive power of each module to zero while maintaining module headings.
     */
    public void stop() {
        drive(0.0, 0.0, 0.0, isFieldOriented);
    }

    /**
     * Turns the modules to the X-lock position as long as drive inputs are zero.
     */
    public void lock() {
        isLocked = true;
    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Controls the lienar and rotational values for the modules based on the free speed of the drive motors (open-loop).
     * 
     * @param moduleStates An array of module states to set. The order is FL, FR, BL, BR.
     */
    public void setModuleStatesOpenLoop(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], false);
        frontRightMod.setDesiredState(moduleStates[1], false);
        backLeftMod.setDesiredState(moduleStates[2], false);
        backRightMod.setDesiredState(moduleStates[3], false);
    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Uses PID and feedforward control (closed-loop) to control the linear and rotational values for the modules.
     * 
     * @param moduleStates An array module states to set. The order is FL, FR, BL, BR.
     */
    public void setModuleStatesClosedLoop(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], true);
        frontRightMod.setDesiredState(moduleStates[1], true);
        backLeftMod.setDesiredState(moduleStates[2], true);
        backRightMod.setDesiredState(moduleStates[3], true);
    }

    /**
     * Returns the current motion of the drive base as a ChassisSpeeds.
     * 
     * @return A ChassisSpeeds representing the current motion of the drive base.
     */
    public ChassisSpeeds getChassisSpeeds() {
        double xVel = getAverageDriveVelocityMetersPerSec() * getDirectionOfTravel().getCos();
        double yVel = getAverageDriveVelocityMetersPerSec() * getDirectionOfTravel().getSin();
        double omega = Units.degreesToRadians(-imu.getRate());

        return new ChassisSpeeds(xVel, yVel, omega);
    }

    /**
     * Sets the ChassisSpeeds of the drive base.
     * 
     * @param chassisSpeeds The desired ChassisSpeeds.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStatesClosedLoop(DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds));
    }
    

    /**
     * Returns an array of module states of the drive base. The order is FL, FR, BL, BR.
     * 
     * @return An array of SwerveModuleState.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeftMod.getVelocityMetersPerSec(), frontLeftMod.getSteerEncAngle()),
            new SwerveModuleState(frontRightMod.getVelocityMetersPerSec(), frontRightMod.getSteerEncAngle()),
            new SwerveModuleState(backLeftMod.getVelocityMetersPerSec(), backLeftMod.getSteerEncAngle()),
            new SwerveModuleState(backRightMod.getVelocityMetersPerSec(), backRightMod.getSteerEncAngle())
        };
    }

    /**
     * Returns an array of module positions.
     * 
     * @return An array of SwerveModulePosition.
     */
    public static SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftMod.getPosition(),
            frontRightMod.getPosition(),
            backLeftMod.getPosition(),
            backRightMod.getPosition()
        };
    }

    /**
     * @return The current estimated position of the robot on the field
     * based on drive encoder and gyro readings.
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Resets the current pose to (0, 0) with a heading of zero.
     */
    public void resetPose(Pose2d pose) {
        resetDriveDistances();
        resetHeading();

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            new Rotation2d(),
            getModulePositions(),
            pose
        );
    }

    public static void setHeading(Rotation2d heading) {
        imu.setYaw(Math.abs(heading.getDegrees() % 360));
    }

    /**
     * Sets the pose of the robot.
     * 
     * @param pose The pose to set the robot to.
     */
    public static void setPose(Pose2d pose) {
        setHeading(pose.getRotation());

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            pose.getRotation(),
            getModulePositions(),
            pose
        );
    }

    public void setTranslation(Translation2d translation) {
        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d(translation, getHeading())
        );
    }

    /**
     * Resets the measured distance driven for each module to zero.
     * <p>Resets the drive encoders of each module to zero.
     */
    public void resetDriveDistances() {
        frontLeftMod.resetDriveDistance();
        frontRightMod.resetDriveDistance();
        backLeftMod.resetDriveDistance();
        backRightMod.resetDriveDistance();
    }

    /**
     * Returns the average distance driven of each module to get an overall distance driven by the robot.
     * 
     * @return The overall distance driven by the robot in meters.
     */
    public double getAverageDriveDistanceMeters() {
        return (
            (frontLeftMod.getDriveDistanceMeters()
            + frontRightMod.getDriveDistanceMeters()
            + backLeftMod.getDriveDistanceMeters()
            + backRightMod.getDriveDistanceMeters())
            / 4.0
        );
    }

    /**
     * Returns the average velocity of each module to get an overall velocity of the robot.
     * 
     * @return The overall velocity of the robot in meters per second.
     */
    public double getAverageDriveVelocityMetersPerSec() {
        return (
            (Math.abs(frontLeftMod.getVelocityMetersPerSec())
            + Math.abs(frontRightMod.getVelocityMetersPerSec())
            + Math.abs(backLeftMod.getVelocityMetersPerSec() )
            + Math.abs(backRightMod.getVelocityMetersPerSec()))
            / 4.0
        );
    }

    /**
     * Returns the average direction of each module to get an overall direction of travel of the robot.
     * 
     * @return The overall direction of travel of the robot.
     */
    public Rotation2d getDirectionOfTravel() {
        return new Rotation2d(
            (frontLeftMod.getCanCoderAngle().plus(new Rotation2d(frontLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + frontRightMod.getCanCoderAngle().plus(new Rotation2d(frontRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + backLeftMod.getCanCoderAngle().plus(new Rotation2d(backLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + backRightMod.getCanCoderAngle().plus(new Rotation2d(backRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            ) / 4.0
        );
    }

    /**
     * Returns the average velocity in the direction relative to the robot.
     * 
     * @param relativeHeading The relative heading of the robot, where zero is the front of the robot.
     * 
     * @return The velocity in the direction relative to the robot in meters per second.
     */
    public double getRelativeVelocityMetersPerSec(Rotation2d relativeHeading) {
        return getDirectionOfTravel().minus(relativeHeading).getCos() * getAverageDriveVelocityMetersPerSec();
    }

    /**
     * Returns the current heading of the robot from the gyro.
     * 
     * @return The current heading of the robot as a Rotation2d.
     */
    public static Rotation2d getHeading() {
        //return Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(-imu.getYaw().getValueAsDouble())));
        return Rotation2d.fromDegrees(imu.getYaw().getValueAsDouble());
    }

    /**
     * Returns the current pitch of the robot from the gyro.
     * 
     * @return The current pitch of the robot as a Rotation2d.
     */
    public Rotation2d getPitch() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return Rotation2d.fromDegrees(imu.getRoll().getValueAsDouble());
    }

    /**
     * Returns the current roll of the robot from the gyro.
     * 
     * @return The current roll of the robot as a Rotation2d.
     */
    public Rotation2d getRollDegrees() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return Rotation2d.fromDegrees(imu.getPitch().getValueAsDouble());
    }

    /**
     * Sets the gyro heading to zero.
     */
    public void resetHeading() {
        imu.setYaw(0.0);
    }

    /**
     * Sets the current limit of the drive motors of each module to the desired amperage.
     * 
     * @param amps The desired current limit of the drive motors in amps.
     */
    public void setDriveCurrentLimit(int amps) {
        frontLeftMod.setDriveCurrentLimit(amps);
        frontRightMod.setDriveCurrentLimit(amps);
        backLeftMod.setDriveCurrentLimit(amps);
        backRightMod.setDriveCurrentLimit(amps);
    }

    /**
     * Enables or disables Turtle Mode based on the current speedFactor value.
     */
    public void setTurtleMode() {
        if (speedFactor == 1)
            speedFactor = 0.3;
        else 
            speedFactor = 1;
        for (int i = 0; i < 4; i++) ;
    }
    public boolean PathFlip() {
        return true;
    }


    public void BuilderConfigure() {
        AutoBuilder.configureHolonomic(
            this:: getPose, 
            this:: resetPose, 
            this:: getChassisSpeeds, 
            this:: setChassisSpeeds, 
             new HolonomicPathFollowerConfig(
                new PIDConstants(1, 0, 0), 
                new PIDConstants(1, 0, 0), 
                DriveConstants.maxDriveSpeedMetersPerSec, 
                DriveConstants.driveBaseRadius, 
                new ReplanningConfig(true, false)), 
            () -> RobotContainer.isRedAlliance(), 
            this
        );
    }
}