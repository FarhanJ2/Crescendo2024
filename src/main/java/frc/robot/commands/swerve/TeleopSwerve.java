package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {    
    // private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier alignSpeakerSup;
    private BooleanSupplier rampFerrySup;

    public TeleopSwerve( 
                        DoubleSupplier translationSup, 
                        DoubleSupplier strafeSup, 
                        DoubleSupplier rotationSup, 
                        BooleanSupplier robotCentricSup, 
                        BooleanSupplier alignSpeakerSup,
                        BooleanSupplier rampFerrySup) {

        // this.s_Swerve = s_Swerve;
        addRequirements(RobotContainer.s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.alignSpeakerSup = alignSpeakerSup;
        this.rampFerrySup = rampFerrySup;
    }

    private double continuous180To360(double angle) {
        return (angle + 360) % 360;
    }

    private double getRotationSpeedFromPID(Pose2d target) {
        double robotHeading = continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees());
        double requestedAngle = RobotContainer.s_Swerve.calculateTurnAngle(target, RobotContainer.s_Swerve.getHeading().getDegrees() + 180);
        double setpoint = (robotHeading + requestedAngle) % 360;

        RobotContainer.s_Swerve.getAlignController().setSetpoint(setpoint);

        return (RobotContainer.s_Swerve.isLowGear() ? 5 : 1) * RobotContainer.s_Swerve.getAlignController().calculate(continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees()));
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Deadbands.driveDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Deadbands.driveDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Deadbands.driveDeadband);

        double multipliedRotation = 0;
        Translation2d multipliedTranslation = new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed);

        // Align to speaker
        if (alignSpeakerSup.getAsBoolean()) {
            multipliedRotation = getRotationSpeedFromPID(RobotContainer.alliance == DriverStation.Alliance.Blue ? Constants.BlueTeamPoses.blueSpeakerPose : Constants.RedTeamPoses.redSpeakerPose);
            multipliedTranslation = RobotContainer.s_Swerve.isLowGear() ? multipliedTranslation : multipliedTranslation.times(0.3);

        // Align to amp
        } else if (rampFerrySup.getAsBoolean()) {
            multipliedRotation = getRotationSpeedFromPID(RobotContainer.alliance == DriverStation.Alliance.Blue ? Constants.BlueTeamPoses.blueAmpPose : Constants.RedTeamPoses.redAmpPose);
        
        // Normal drive
        } else {
            multipliedRotation = rotationVal * Constants.Swerve.maxAngularVelocity;
        }

        /* Drive */
        RobotContainer.s_Swerve.drive(
            multipliedTranslation,
            multipliedRotation, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}