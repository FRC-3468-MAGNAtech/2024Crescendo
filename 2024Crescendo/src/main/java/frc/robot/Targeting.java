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

	public static double aimArmToSpeakerInt() {
		double shootSpeed = RobotContainer.m_camera.speedMap.get(Camera.getTZ());
		if (shootSpeed < 0.7)
			shootSpeed = 0.7;
		if (shootSpeed > 0.9)
			shootSpeed = 0.9;
		RobotContainer.m_shooter.setSpeed = shootSpeed;
		double angle = RobotContainer.m_camera.angleMap.get(Camera.getTZ());
		if (angle > 0.47) angle = 0.47;
		return angle;
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
