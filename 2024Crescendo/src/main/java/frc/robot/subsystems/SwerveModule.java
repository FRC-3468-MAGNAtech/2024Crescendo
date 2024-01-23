package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.config.OnboardModuleStates;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;
    private final DutyCycleEncoder angleEncoderTmp;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward feedforward = 
        new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = Rotation2d.fromDegrees(moduleConstants.CANcoderOffset);
        
        /* Configuring Angle Encoding */
        angleEncoder  = new CANcoder(moduleConstants.CANcoderId);
        ConfigureAngleEncoder(false); //for right now
        angleEncoderTmp = new DutyCycleEncoder(moduleConstants.CANcoderId);
        angleEncoderTmp.setPositionOffset(angleOffset.getDegrees() / 360);

        /* Configuring Angle Motor */
        angleMotor = new CANSparkMax(moduleConstants.SteerMotorId, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        ConfigureAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.DriveMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        ConfigureDriveMotor();

        lastAngle = getState().angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
          double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
          driveMotor.set(percentOutput);
        } else {
          driveController.setReference(
              desiredState.speedMetersPerSecond,
              ControlType.kVelocity,
              0,
              feedforward.calculate(desiredState.speedMetersPerSecond));
        }
      }

    private void resetToAbsolute() {
        double absolutePos = angleEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePos);
    }

    private void ConfigureAngleEncoder(boolean invert) {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoderConfigurator conf = angleEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
        conf.refresh(magnetSensorConfig);
        conf.apply(magnetSensorConfig
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(invert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive));
    }

    private void ConfigureAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        //can bus slow down
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.Swerve.angleInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKFF);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void ConfigureDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        //max bus usage
        driveMotor.setOpenLoopRampRate(.1);
        driveMotor.setClosedLoopRampRate(.1);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor((Constants.Swerve.driveConversionVelocityFactor));
        driveEncoder.setPositionConversionFactor((Constants.Swerve.driveConversionPositionFactor));
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKFF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }



    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            getAngle()
        );
    }

    public void SetDriveIdle(IdleMode Idle) {
        driveMotor.setIdleMode(Idle);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = OnboardModuleStates.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
}
