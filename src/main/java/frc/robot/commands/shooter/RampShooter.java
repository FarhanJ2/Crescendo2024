package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RampShooter extends Command {

    // private Supplier<Double> distanceSupplier;
    private double topRPM;
    private double bottomRPM;
    private double angle;

    public RampShooter(double topRPM, double bottomRPM, double angle) {
        // this.distanceSupplier = distanceSupplier;
        this.topRPM = topRPM;
        this.bottomRPM = bottomRPM;
        this.angle = angle;

        addRequirements(RobotContainer.s_Shooter);
    }

    //  implement
    // public double calculateRPM(double distance) {
    //     return 0;
    // }

    @Override
    public void execute() {
        RobotContainer.s_Shooter.rampShooter(topRPM, bottomRPM);
        RobotContainer.s_Shooter.setPivot(angle);

        // calculateRPM(distanceSupplier.get())
    }

    @Override
    public boolean isFinished() {
        // return RobotContainer.s_Shooter.shooterAtSetpoint();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Shooter.stopShooter();
    }
}
