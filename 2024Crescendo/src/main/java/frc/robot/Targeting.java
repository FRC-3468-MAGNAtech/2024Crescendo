package frc.robot;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.armConstants;
import frc.robot.subsystems.Camera;

public final class Targeting {
	

	//InterpolatingMatrixTreeMap treeMap = new InterpolatingMatrixTreeMap<>().put(, );

	public static double aimToAprilTag() {
		double tx = LimelightHelpers.getTX(LimelightConstants.llTags);
		return -LimelightConstants.llPIDctrlRotate.calculate(tx);
    }

	public static double aimArmToSpeaker() {
		return armConstants.shooterEquationM * Camera.getTZ() + armConstants.shooterEquationB;
    }

	public static double aimArmToSpeakerInt() {
		double shootSpeed = RobotContainer.m_camera.speedMap.get(Camera.getTZ());
		if (shootSpeed < 0.65)
			shootSpeed = 0.65;
		RobotContainer.m_shooter.setSpeed = shootSpeed;
		return RobotContainer.m_camera.angleMap.get(Camera.getTZ());
    }

	public static double driveToNote() {
		double ta = LimelightHelpers.getTA(LimelightConstants.llNotes);
		return LimelightConstants.llPIDctrlDrive.calculate(ta);
	}

	public static double aimToNote() {
		double tx = LimelightHelpers.getTX(LimelightConstants.llNotes);
		return -LimelightConstants.llPIDctrlRotate.calculate(tx);
	}
}
