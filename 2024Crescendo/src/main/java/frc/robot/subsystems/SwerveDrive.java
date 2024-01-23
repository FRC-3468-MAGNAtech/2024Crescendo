package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private SwerveDriveKinematics swerveDriveKinematics;
    private Field2d field;
    public  int automode;

    private boolean turtleToggle;
    public double speed;

    public SwerveDrive() {
        turtleToggle = false;
        speed = Constants.Swerve.maxSpeed;

        gyro = new Pigeon2(0);

        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.FrontLeftSwerveModule.Constants),
            new SwerveModule(1, Constants.Swerve.FrontRightSwerveModule.Constants),
            new SwerveModule(2, Constants.Swerve.BackLeftSwerveModule.Constants),
            new SwerveModule(3, Constants.Swerve.BackRightSwerveModule.Constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(
        Translation2d translation, double rotataion, boolean fieldRelative, boolean isOpenLoop
    ) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotataion, getYaw())
            :   new ChassisSpeeds(translation.getX(), translation.getY(), rotataion)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);
        for (SwerveModule mod : mSwerveMods)
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, speed);

        for (SwerveModule mod : mSwerveMods)
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void drive(Translation2d translation)
}