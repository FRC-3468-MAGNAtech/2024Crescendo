package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeNote extends Command {
		Intake intakeSys;
	/** Creates a new AutoIntakeNote. */
	public AutoIntakeNote(Intake sys) {
		// Use addRequirements() here to declare subsystem dependencies.
		intakeSys = sys;
		addRequirements(intakeSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		intakeSys.intake();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intakeSys.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return intakeSys.getIntakeSensor();
	}
}
