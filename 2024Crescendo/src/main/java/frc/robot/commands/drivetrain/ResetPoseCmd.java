package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class ResetPoseCmd extends Command {

    private final SwerveSys swerveSys;


    public ResetPoseCmd(SwerveSys swerveSys) {

        this.swerveSys = swerveSys;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        swerveSys.resetPose(new Pose2d());

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
