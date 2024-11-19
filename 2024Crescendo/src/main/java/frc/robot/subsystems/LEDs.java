// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
	public static enum LEDColor {
		Black
	,	White
	,	Red
	,	Green
	,	Blue
	,	Magenta
	,	Yellow
	,	Cyan
	}
	private final Solenoid redLED;
	private final Solenoid greenLED;
	private final Solenoid blueLED;
	private final Solenoid whiteLED;
	private final Solenoid redLED2;
	private final Solenoid greenLED2;
	private final Solenoid blueLED2;
	private final Solenoid whiteLED2;
	private final Solenoid shooterLEDRed;
	private final Solenoid shooterLEDBlue;

	private LEDColor currentColor = LEDColor.Black;
	/** Creates a new LEDs. */
	public LEDs() {
		//climber colors
		redLED = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDRed);
		greenLED = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDGreen);
		blueLED = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDBlue);
		whiteLED = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDWhite);
		redLED2 = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDRed2);
		greenLED2 = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDGreen2);
		blueLED2 = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDBlue2);
		whiteLED2 = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.LEDWhite2);
		//shooter colors
		shooterLEDRed = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.ShooterLEDRed);
		shooterLEDBlue = new Solenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, LEDConstants.ShooterLEDBlue);
		clearColor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	private void enableRed(boolean enable) {
		redLED.set(enable);
		redLED2.set(enable);
	}

	private void enableGreen(boolean enable) {
		greenLED.set(enable);
		greenLED2.set(enable);
	}

	private void enableBlue(boolean enable) {
		blueLED.set(enable);
		blueLED2.set(enable);
	}

	private void enableWhite(boolean enable) {
		whiteLED.set(enable);
		whiteLED2.set(enable);
	}

	private void makeItRed() {
		enableRed(true);
		enableGreen(false);
		enableBlue(false);
		enableWhite(false);
	}

	private void makeItGreen() {
		enableRed(false);
		enableGreen(true);
		enableBlue(false);
		enableWhite(false);
	}

	private void makeItBlue() {
		enableRed(false);
		enableGreen(false);
		enableBlue(true);
		enableWhite(false);
	}

	private void makeItMagenta() {
		enableRed(true);
		enableGreen(false);
		enableBlue(true);
		enableWhite(false);
	}

	private void makeItCyan() {
		enableRed(false);
		enableGreen(true);
		enableBlue(true);
		enableWhite(false);
	}

	private void makeItYellow() {
		enableRed(true);
		enableGreen(true);
		enableBlue(false);
		enableWhite(false);
	}

	private void makeItWhite() {
		enableRed(false);
		enableGreen(false);
		enableBlue(false);
		enableWhite(true);
	}

	private void clearColor() {
		enableRed(false);
		enableGreen(false);
		enableBlue(false);
		enableWhite(false);
	}

	public void setShooterAllianceColor() {
		if (RobotContainer.isRedAlliance()) {
			shooterLEDBlue.set(false);
			shooterLEDRed.set(true);
		} else {
			shooterLEDBlue.set(true);
			shooterLEDRed.set(false);
		}
	}
	
	public void setColor(LEDColor color) {
		boolean colorChanged = color != currentColor;
		currentColor = color;
		if (!colorChanged) return;
		switch (color) {
			case Red:
				makeItRed();
				break;
			case Green:
				makeItGreen();
				break;
			case Blue:
				makeItBlue();
				break;
			case White:
				makeItWhite();
				break;
			case Cyan:
				makeItCyan();
				break;
			case Yellow:
				makeItYellow();
				break;
			case Magenta:
				makeItMagenta();
				break;
			case Black:
				clearColor();
				break;
			default:
				break;
		}
	}
}
