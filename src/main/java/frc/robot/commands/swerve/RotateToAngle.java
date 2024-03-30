package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAngle extends Command {    
    private Supplier<Double> requestedAngle;
    private Supplier<Boolean> buttonPressed;
    boolean finished = false;

    private final PIDController alignPID = new PIDController(
        Constants.Swerve.autoAlignKP,
        Constants.Swerve.autoAlignKI,
        Constants.Swerve.autoAlignKD
    );

    public RotateToAngle(Supplier<Double> requestedAngle, Supplier<Double> currentAngle) {
        this(requestedAngle, currentAngle, () -> true);
    }

    public RotateToAngle(Supplier<Double> requestedAngle, Supplier<Double> currentAngle, Supplier<Boolean> buttonPressed) {
        
        alignPID.enableContinuousInput(0, 360);
        alignPID.setTolerance(1);
        
        this.buttonPressed = buttonPressed;
        this.requestedAngle = requestedAngle;

        addRequirements(RobotContainer.s_Swerve);
    }

    @Override
    public void initialize() {
        double robotHeading = continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees());
        double setpoint = (robotHeading + requestedAngle.get()) % 360;

        alignPID.setSetpoint(setpoint);
    }

    private double continuous180To360(double angle) {
        return (angle + 360) % 360;
    }

    @Override
    public void execute() {
        RobotContainer.s_Swerve.drive(
            new Translation2d(),
            (RobotContainer.s_Swerve.isLowGear() ? 5 : 1) * alignPID.calculate(continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees())),
            true,
            false
        );
    }

    @Override
    public boolean isFinished() {
        if(buttonPressed == null) return alignPID.atSetpoint();
        else return (alignPID.atSetpoint() && !buttonPressed.get())
            || !buttonPressed.get();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Swerve.stop();
    }
}