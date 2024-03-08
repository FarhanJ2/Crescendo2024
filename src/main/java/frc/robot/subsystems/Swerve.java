package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import java.sql.Driver;

import frc.robot.SwerveModule;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PoseConfig;
import frc.lib.util.Limelight;
import frc.lib.util.OdometryImpl;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    // public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator; 
    public SwerveModule[] mSwerveMods;
    public OdometryImpl odometryImpl;
    public Pigeon2 gyro;

    public Field2d field;

    public Limelight limelightShooter;
    public Limelight limelightArm;

    public DriverStation.Alliance alliance; 

    private SendableChooser limelightChooser; 

    private ChassisSpeeds chassisSpeeds;

    private double speedMultiplier = 1;

    private TalonFX m_frontLeftMotor;
    private TalonFX m_frontRightMotor;
    private TalonFX m_backLeftMotor;
    private TalonFX m_backRightMotor;

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    // SysID
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    private final Measure<Velocity<Voltage>> m_desiredRampRate = Velocity.combine(Volts, Second).of(1);
    private final Measure<Voltage> m_desiredStepVoltage = Volts.of(5);


    //TODO CODE NOT CLEAN
    public final Thread poseEstimatorInitializer = new Thread(() -> {
        DriverStation.reportWarning("ISAAC WONG", false);
        if (RobotContainer.alliance == DriverStation.Alliance.Blue) {
            poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            Constants.BlueTeamPoses.blueOrigin,
            odometryImpl.createStdDevs(PoseConfig.kPositionStdDevX, PoseConfig.kPositionStdDevY, PoseConfig.kPositionStdDevTheta),
            odometryImpl.createStdDevs(PoseConfig.kVisionStdDevX, PoseConfig.kVisionStdDevY, PoseConfig.kVisionStdDevTheta)
            );
        }

        else {
            poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            Constants.RedTeamPoses.redOrigin,
            odometryImpl.createStdDevs(PoseConfig.kPositionStdDevX, PoseConfig.kPositionStdDevY, PoseConfig.kPositionStdDevTheta),
            odometryImpl.createStdDevs(PoseConfig.kVisionStdDevX, PoseConfig.kVisionStdDevY, PoseConfig.kVisionStdDevTheta)
            );
        }
    });



    // private final SysIdRoutine sysID;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        field = new Field2d();
        odometryImpl = new OdometryImpl(this);
        limelightShooter = new Limelight(Constants.LimelightConstants.limelightShooter);
        limelightArm = new Limelight(Constants.LimelightConstants.limelightArm);
        limelightChooser = new SendableChooser<>();
        limelightChooser.setDefaultOption("None", null);
        limelightChooser.addOption(limelightShooter.getLimelightName(), limelightShooter);
        limelightChooser.addOption(limelightArm.getLimelightName(), limelightArm);

        limelightShooter.setPipeline(LimelightConstants.limelightShooterTagPipeline);
        //limelightArm.setPipeline(LimelightConstants.limelightShooterTagPipeline);

        //TODO FIXXXXXXXXXX plz
        
        m_frontLeftMotor = mSwerveMods[0].getDriveMotor();
        m_frontRightMotor = mSwerveMods[1].getDriveMotor();
        m_backLeftMotor = mSwerveMods[2].getDriveMotor();
        m_backRightMotor = mSwerveMods[3].getDriveMotor();


   

        // sysID = new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         null,
        //         null,
        //         null),
                // (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            // new SysIdRoutine.Mechanism(
            //     (Measure<Voltage> volts) -> {
            //         m_frontLeftMotor.setVoltage(volts.in(Volts));
            //         m_frontRightMotor.setVoltage(volts.in(Volts));
            //         m_backLeftMotor.setVoltage(-volts.in(Volts));
            //         m_backRightMotor.setVoltage(-volts.in(Volts));
            //     },
            //     log -> {
            //         log.motor("swerve")
            //         .voltage(
            //             m_appliedVoltage.mut_replace(
            //                 m_frontLeftMotor.get() * RobotController.getBatteryVoltage(), Volts))
            //         //.angularPosition(m_angle.mut_replace(m_pivotMotor.getPosition().getValue() / 18.8888888888888, Rotations))
            //         .linearPosition(m_distance.mut_replace(m_frontLeftMotor.getPosition().getValueAsDouble() * Math.PI * 0.1016, Meters))
            //         // .angularVelocity(
            //         //     m_velocity.mut_replace(m_pivotMotor.getVelocity().getValue() / 18.8888888888888, RotationsPerSecond));
            //         .linearVelocity(
            //             m_velocity.mut_replace(m_frontLeftMotor.getVelocity().getValueAsDouble() * Math.PI * 0.1016, MetersPerSecond));
            //     },
            //     this));

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(1, 0, 0), // Translation PID constants
                        new PIDConstants(0.095, 0.0009, 0.01), // (p 0.85), (p 0.6 d 0.3),  
                        4.3, // Max module speed, in m/s
                        Constants.Swerve.trackWidth, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }



    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        chassisSpeeds = new ChassisSpeeds(
            translation.getX() * speedMultiplier,
            translation.getY() * speedMultiplier,
            rotation * speedMultiplier
        );
    }    

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public double calculateTurnAngle(Pose2d target, double robotAngle) {
        double tx = target.getX(); 
        double ty = target.getY(); 
        double rx = getPose().getX();
        double ry = getPose().getY();

        double requestedAngle = Math.atan((ty - ry) / (tx - rx)) * (180 / Math.PI);
        double calculatedAngle = (180 - robotAngle + requestedAngle);

        return ((calculatedAngle + 360) % 360);
    }
    

    public void toggleMultiplier() {
        speedMultiplier = speedMultiplier == 1 ? 0.2 : 1;
    }

    public boolean isLowGear() {
        return speedMultiplier == 0.2;
    }

    public Pose2d getPose() {
        //TODO CODE NOT CLEAN
        if(poseEstimator == null) return new Pose2d();
        return poseEstimator.getEstimatedPosition();

    }

    public Pose2d getRelativePose() {
        //TODO CODE NOT CLEAN
        if(poseEstimator == null) return new Pose2d();
        if(RobotContainer.alliance == DriverStation.Alliance.Blue) {
            return poseEstimator.getEstimatedPosition();
        }
        else {
            return poseEstimator.getEstimatedPosition().relativeTo(Constants.RedTeamPoses.redOrigin);
        }
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getRelativePose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        Pose2d zeroPose;
        if(RobotContainer.alliance == DriverStation.Alliance.Blue) {
            zeroPose = new Pose2d(getPose().getTranslation(), new Rotation2d());
        }
        else {
            zeroPose = new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180));
        }
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), zeroPose);    
    }

    public Rotation2d getGyroYaw() {
        // if(Math.abs(gyro.getYaw().getValue()) < 1) return Rotation2d.fromDegrees(0);i
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     // return null;
    //     return sysID.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     // return null;
    //     return sysID.dynamic(direction);
    // }

    public Limelight getFlashedLimelight() {
        return (Limelight) limelightChooser.getSelected(); 
    }


    @Override
    public void periodic(){
        // m_backLeftMotor.setControl(new Follower(Constants.Swerve.Mod0.driveMotorID, false));
        // m_backRightMotor.setControl(new Follower(Constants.Swerve.Mod1.driveMotorID, false));

        // swerveOdometry.update(getGyroYaw(), getModulePositions());
        if (poseEstimator != null) poseEstimator.update(getGyroYaw(), getModulePositions());

        // limelight and odometry classes are written so that adding additional limelights is easy 
    
        // TODO all this code must be uncommented for vision stuff
        // if (Robot.state != Robot.State.AUTON && RobotContainer.addVisionMeasurement) {
        //     Pose2d visionMeasurementLimelightShooter = odometryImpl.getVisionMeasurement(limelightShooter); //changed from without yaw
        //     if (visionMeasurementLimelightShooter != null && poseEstimator != null) {
        //         poseEstimator.addVisionMeasurement(visionMeasurementLimelightShooter, limelightShooter.getLimelightLatency());
        //     }


        //     // //newly added limelight automatically configured for odometry impl
        //     Pose2d visionMeasurementLimelightArm = odometryImpl.getVisionMeasurement(limelightArm); //changed from without yaw
        //     if (visionMeasurementLimelightArm != null && poseEstimator != null) {
        //         poseEstimator.addVisionMeasurement(visionMeasurementLimelightArm, limelightArm.getLimelightLatency());
        //     }
        // }

        // else {
        //     // System.out.println("TELEOP"); 
        // }
        
        // limelights computes the correct pose but it's placed incorrectly on glass (offsetted by 1 meter)
        field.setRobotPose(new Pose2d(getPose().getX() - 1, getPose().getY(), getPose().getRotation()));

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " Voltage", mod.getVoltage());
        }

        SmartDashboard.putNumber("swerve/yaw", gyro.getYaw().getValue());

        // TODO need to change depending on the team your're on
        SmartDashboard.putNumber("swerve/distance", odometryImpl.getDistance(Constants.RedTeamPoses.redSpeakerPose)); 

        SmartDashboard.putData("field", field);
        SmartDashboard.putData("limelightChooser", limelightChooser);

        
    }

    public void stop() {
        RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0), 
            0, true, true
            
        );
    }
    
}