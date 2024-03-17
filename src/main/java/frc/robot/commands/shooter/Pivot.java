package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Pivot extends Command {

    // private Supplier<Double> distanceSupplier;
    private double angle;

    public Pivot(double angle) {
        // this.distanceSupplier = distanceSupplier;
        this.angle = angle;

        addRequirements(RobotContainer.s_Shooter);
    }

    //  implement maybe not
    // public double calculateAngle(double distance) {
    //     return 0;
    // }

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
