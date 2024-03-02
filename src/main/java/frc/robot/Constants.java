package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26.75);
        public static final double wheelBase = Units.inchesToMeters(26.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.05;
        public static final double driveKV = 2.35;
        public static final double driveKA = 0.17;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-100.55);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.27);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.05);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-70.40);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double translationkP = 0;
        public static final double translationkI = 0;
        public static final double translationkD = 0;

        public static final double rotationkP = 0;
        public static final double rotationkI = 0;
        public static final double rotationkD = 0;
    }

    //Subsystems
    public static final class Intake {
        public static final int intakeMotorID = 13;
        public static final int forkMotorID = 14;

        public static final int beamBreakerIntake = 0;
        // public static final int beamBrakerShooter = 2;
        public static final int beamBreakerArm = 3;

        public static final double intakeVoltage = 7;
        public static final double forkVoltage = 7;

        // Manual Values
        public static final double intakeSpeed = 1; // 1
        public static final double forkSpeed = 1;
        public static final double intakeFeedSpeed = 1; // for arm and shooter
    }

    public static final class Shooter {
        public static final int bottomShooterMotorID = 15;
        public static final int topShooterMotorID = 16;

        public static final int feederMotorID = 17;
        
        public static final int pivotMotorID = 18;

        public static final int canCoderID = 19;

        public static final double pivotkS = 0.09256; //0.13884
        public static final double pivotkG = 0.15116; // 0.28674
        public static final double pivotkV = 1.275; // 0.85
        public static final double pivotkA = 0; //1.7626

        public static final double pivotkP = 5; // 5
        public static final double pivotkI = 0;
        public static final double pivotkD = 0;

        public static final double topShooterkS = 0.19655; 
        public static final double topShooterkV = 0.00212586; //0.042517
        public static final double topShooterkA = 0.00025997; //0.015598

        public static final double topShooterkP = 0.01; //0.01
        public static final double topShooterkI = 0;
        public static final double topShooterkD = 0; //0

        public static final double bottomShooterkS = 0.13122;
        public static final double bottomShooterkV = 0.00198405; //0.00066135 //0.039681
        public static final double bottomShooterkA = 0.00071765; //0.014353

        public static final double bottomShooterkP = 0.01; //0.035013
        public static final double bottomShooterkI = 0;
        public static final double bottomShooterkD = 0;

        public static final double pivotTolerance = 0.01; // <1 degree
        public static final double shooterTolerance = 50; // 20 rpm for teleop  ---- 250 rpm for auton

        public static final double homePosition = 1.05; // 1.05

        public static final double feedSpeed = 0.85;

        // Manual testing values
        public static final double topShooterSpeed = 0.4; // 1
        public static final double bottomShooterSpeed = 0.4; // 1
        public static final double manualShooterPivotSpeed = 0.05;

        //TODO find max rpm
        public static final double topShooterMaxRPM = 4500; 
        public static final double bottomShooterMaxRPM = 4800;

        public static final double kMaxVelocityRadPerSecond = 6;
        public static final double kMaxAccelerationRadPerSecSquared = 8;
    }

    public static final class Elevator {
        public static final int leftMotorID = 20;
        public static final int rightMotorID = 21;

        public static final int limitSwitchLowerChannel = 2;

        public static final int canCoderID = 22;

        public static final double kS = 0.15;
        public static final double kG = 0;
        public static final double kV = 5.65;

        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kMaxVelocityPerSecond = 2;
        public static final double kMaxAccelerationPerSecSquared = 6;

        public static final double tolerance = 0.05;

        public static final double climbRotations = 2.8;
        public static final double ampRotations = 2.3;
        public static final double homeRotations = 0;
        public static final double trapRotations = 1.5;

        public static final double manualElevatorSpeed = 0.8;
    }

    public static final class AmpArm {
        public static final int pivotMotorID = 22;
        public static final int shootMotorID = 23;

        public static final int canCoderID = 24;        
        public static final double pivotkS = 0.14; //0.14 //0.065414
        public static final double pivotkG = 0.095; //0.095
        public static final double pivotkV = 0.46; //0.46
        // public static final double pivotkA = 0.3;

        public static final double pivotkP = 1; // 1 // 10
        public static final double pivotkI = 0;
        public static final double pivotkD = 0; //0.000001 // 0.06

        public static final double integratorZone = 0;

        public static final double pivotTolerance = 0.01;

        public static final double kMaxVelocityRadPerSecond = 1;
        public static final double kMaxAccelerationRadPerSecSquared = 8;

        public static final double armOffset = -Math.PI / 2; // -2.13

        // public static final double shootSpeed = 0;

        // Score positions in radians
        public static final double homePosition = -Math.PI / 2; // -2.37
        public static final double handoffPosition = -2.22;
        public static final double ampSlamPosition = -0.20; // 0.75
        public static final double ampShootPosition = -0.6;
        public static final double trapPosition = 1.37;

        // Manual testing values
        public static final double manualArmPivotSpeed = 0.05;
        public static final double handoffSpeed = -0.15;
        public static final double shootSpeed = 0.3;
    }

    public static final class Vision {
        public static final double threshold_distance = 0.2; 
        public static final double y_vel_constant = 54;
        public static final double threshold_tx_offset = 0.3;
        public static double kMaxTrackerDistance;
        public static double kMaxGoalTrackAge;
        public static double kMaxGoalTrackSmoothingTime;
        public static int kCameraFrameRate; 
    }

    public static final class Led {
        public static final int port = 0;
        public static final int length = 60;
    }

    public static class LimelightConstants {
        // All limelights on the robot
        public static final String limelightShooter = "limelight-shooter";
        public static final String limelightArm = "limelight-arm";

        // Tracking constants
        public static final double minAreaOfTag = .1;
        public static final double maxVisionPoseError = 0.5;

        // Pipeline IDS 
        public static final int limelightShooterTagPipeline = 0;
        public static final int limelightArmTagPipeline = 0;

    }

    public static class PoseConfig {
        // Increase these numbers to trust your model's state estimates less.
        public static final double kPositionStdDevX = 0.1;
        public static final double kPositionStdDevY = 0.1;
        public static final double kPositionStdDevTheta = 10;

        // Increase these numbers to trust global measurements from vision less.
        public static final double kVisionStdDevX = 5;
        public static final double kVisionStdDevY = 5;
        public static final double kVisionStdDevTheta = 500;    
      }

    public static class BlueTeamPoses {
        // Blue team poses
        public static final Pose2d initialPose = new Pose2d(new Translation2d(1.225347, 7.652309), new Rotation2d()); // Next to blue speaker
        public static final Pose2d blueSpeakerPose = new Pose2d(new Translation2d(1.286580, 5.671689), Rotation2d.fromDegrees(180));
        public static final Pose2d redSpeakerPose = new Pose2d(new Translation2d(16.541748, 5.700184), new Rotation2d());
        public static final Pose2d blueOrigin = new Pose2d(new Translation2d(0, 0), new Rotation2d()); 
        
    }

    public static class RedTeamPoses {
        // TODO: add the red team poses which just be the blue team poses for structures but inverted
        public static final Pose2d initialPose = new Pose2d(new Translation2d(1.225347, 7.652309), new Rotation2d()); // Next to blue speaker
        public static final Pose2d blueSpeakerPose = new Pose2d(new Translation2d(16.541748, 5.700184), Rotation2d.fromDegrees(180));
        public static final Pose2d redSpeakerPose = new Pose2d(new Translation2d(1.286580, 5.671689), Rotation2d.fromDegrees(0)); 
        public static final Pose2d redOrigin = new Pose2d(new Translation2d(16.542, 8.014), Rotation2d.fromDegrees(180)); 

    }

    public static final class ShootingConstants {
      public static final double podiumRPM = 0; // 3000
      public static final double podiumAngle = 0; // 0.7

      public static final double speakerRPM = 0; // 1500
      public static final double speakerAngle = 1.17;
      public static final Pose2d redOrigin = new Pose2d(new Translation2d(16.542, 8.014), Rotation2d.fromDegrees(180)); 
      
  }

}
