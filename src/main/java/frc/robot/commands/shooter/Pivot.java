package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Pivot extends Command {

    private double angle;

    public Pivot(double angle) {
        this.angle = angle;

        addRequirements(RobotContainer.s_Shooter);
    }

    @Override
    public void execute() {
        RobotContainer.s_Shooter.setPivot(angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Shooter.stopPivot();
    }
}
