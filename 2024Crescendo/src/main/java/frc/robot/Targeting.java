package frc.robot;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.armConstants;
import frc.robot.subsystems.Camera;

public final class Targeting {

	public static double aimToAprilTag() {
		double tx = LimelightHelpers.getTX(LimelightConstants.llTags);
		return -LimelightConstants.llPIDctrlRotate.calculate(tx);
    }

	public static double aimArmToSpeaker() {
		double feet = Camera.getDistance() / 12;
		if (feet >= 3 || feet < 6.5)
			return armConstants.upPIDReference0;
		if (feet >= 6.5 || feet < 10)
			return armConstants.upPIDReference3_5;
		if (feet >= 10 || feet < 13)
			return armConstants.upPIDReference7;
		
		return armConstants.upPIDReference10;
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
