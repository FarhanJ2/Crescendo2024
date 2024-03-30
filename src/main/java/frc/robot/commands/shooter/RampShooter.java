package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RampShooter extends Command {

    private double topRPM;
    private double bottomRPM;
    private double angle;

    public RampShooter(double topRPM, double bottomRPM, double angle) {
        this.topRPM = topRPM;
        this.bottomRPM = bottomRPM;
        this.angle = angle;

        addRequirements(RobotContainer.s_Shooter);
    }

    @Override
    public void execute() {
        RobotContainer.s_Shooter.rampShooter(topRPM, bottomRPM);
        RobotContainer.s_Shooter.setPivot(angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Shooter.stopShooter();
    }
}
