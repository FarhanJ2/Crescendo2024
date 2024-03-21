package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier alignSpeakerSup;

    private final PIDController alignPID = new PIDController( // TODO fix this
        0.07,
        0,
        0
    );

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier alignSpeakerSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.alignSpeakerSup = alignSpeakerSup;

        alignPID.enableContinuousInput(0, 360);
        alignPID.setTolerance(1);
    }

    private double continuous180To360(double angle) {
        return (angle + 360) % 360;
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double multipliedRotation = 0;
        Translation2d multipliedTranslation = new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed);

        if (!alignSpeakerSup.getAsBoolean()) { // Drive normally
            multipliedRotation = rotationVal * Constants.Swerve.maxAngularVelocity;
        } else { // Drive with alignment
            double robotHeading = continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees());
            double requestedAngle = s_Swerve.calculateTurnAngle(RobotContainer.alliance == DriverStation.Alliance.Blue ? Constants.BlueTeamPoses.blueSpeakerPose : Constants.RedTeamPoses.redSpeakerPose, s_Swerve.getHeading().getDegrees() + 180);
            double setpoint = (robotHeading + requestedAngle) % 360;

            alignPID.setSetpoint(setpoint);

            multipliedRotation = (RobotContainer.s_Swerve.isLowGear() ? 5 : 1) * alignPID.calculate(continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees()));
            multipliedTranslation = RobotContainer.s_Swerve.isLowGear() ? multipliedTranslation : multipliedTranslation.times(0.3);
        }

        /* Drive */
        s_Swerve.drive(
            multipliedTranslation,
            multipliedRotation, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}