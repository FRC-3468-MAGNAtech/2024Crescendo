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
		return armConstants.shooterEquationM * Camera.getArea() + armConstants.shooterEquationB;
		
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
