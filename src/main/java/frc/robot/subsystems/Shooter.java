package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.Conversions;
import frc.lib.util.NoteVisualizer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.feeder.FeedAmp;
import frc.robot.commands.feeder.FeedtoIntake;
import frc.robot.commands.intake.AmpForkCommand;
import frc.robot.commands.intake.ForkCommand;
import frc.robot.commands.intake.ForktoIntakeCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.ManualShot;
import frc.robot.commands.shooter.Pivot;
import frc.robot.commands.shooter.RampAmp;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.regression.ShooterInterpolator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Direction;
import frc.robot.subsystems.LED.LEDColor;
import frc.team254.lib.util.InterpolatingDouble;

public class Shooter extends ProfiledPIDSubsystem {
    private final TalonFX m_shooterBottomMotor = new TalonFX(Constants.Shooter.bottomShooterMotorID, Constants.canivoreName);
    private final TalonFX m_shooterTopMotor = new TalonFX(Constants.Shooter.topShooterMotorID, Constants.canivoreName);
    private final TalonFX m_pivotMotor = new TalonFX(Constants.Shooter.pivotMotorID, Constants.canivoreName);

    private final CANcoder m_cancoder = new CANcoder(Constants.Shooter.canCoderID, Constants.canivoreName);

    public boolean isShooting = false;
    public boolean isScoringAmp = false;

    public double trigTargetAngle = 0;

    /* Syd ID */
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private final Measure<Velocity<Voltage>> m_desiredRampRate = Volts.of(0.3).per(Seconds.of(1));
    private final Measure<Voltage> m_desiredStepVoltage = Volts.of(1);

    //Create a new SysId routine for characterizing the shooter.
    // private final SysIdRoutine m_sysIdRoutine =
    //   new SysIdRoutine(
    //       // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
    //       new  SysIdRoutine.Config(m_desiredRampRate, m_desiredStepVoltage, null),
    //       new SysIdRoutine.Mechanism(
    //           // Tell SysId how to plumb the driving voltage to the motor(s).
    //           (Measure<Voltage> volts) -> {
    //             m_pivotMotor.setVoltage(volts.in(Volts));
    //           },
    //           // Tell SysId how to record a frame of data for each motor on the mechanism being
    //           // characterized.
    //           log -> {
    //             // Record a frame for the shooter motor.
    //             log.motor("shooter")
    //                 .voltage(
    //                     m_appliedVoltage.mut_replace(
    //                         m_pivotMotor.get() * RobotController.getBatteryVoltage(), Volts))
    //                 //.angularPosition(m_angle.mut_replace(m_pivotMotor.getPosition().getValue() / 18.8888888888888, Rotations))
    //                 .angularPosition(m_angle.mut_replace(getMeasurement(), Radians))
    //                 // .angularVelocity(
    //                 //     m_velocity.mut_replace(m_pivotMotor.getVelocity().getValue() / 18.8888888888888, RotationsPerSecond));
    //                 .angularVelocity(
    //                     m_velocity.mut_replace(m_pivotMotor.getVelocity().getValueAsDouble() / 111.51515151515151 * 2 * Math.PI, RadiansPerSecond));
    //           },
    //           // Tell SysId to make generated commands require this subsystem, suffix test state in
    //           // WPILog with this subsystem's name ("shooter")
    //           this));

    private final PIDController bottomShooterPIDController = 
        new PIDController(
            Constants.Shooter.bottomShooterkP, 
            Constants.Shooter.bottomShooterkI, 
            Constants.Shooter.bottomShooterkD
        );

    private final PIDController topShooterPIDController = 
        new PIDController(
            Constants.Shooter.topShooterkP, 
            Constants.Shooter.topShooterkI, 
            Constants.Shooter.topShooterkD
        );
    
    private final ArmFeedforward pivotFeedforward =
        new ArmFeedforward(
            Constants.Shooter.pivotkS, 
            Constants.Shooter.pivotkG,
            Constants.Shooter.pivotkV
    );

    private final SimpleMotorFeedforward topShooterFeedforward = 
        new SimpleMotorFeedforward(
            Constants.Shooter.topShooterkS,
            Constants.Shooter.topShooterkV
        );

    private final SimpleMotorFeedforward bottomShooterFeedforward = 
        new SimpleMotorFeedforward(
            Constants.Shooter.bottomShooterkS,
            Constants.Shooter.bottomShooterkV
        );

    public Shooter() {
        super(new ProfiledPIDController(
            Constants.Shooter.pivotkP, 
            Constants.Shooter.pivotkI, 
            Constants.Shooter.pivotkD, 
            new TrapezoidProfile.Constraints(
                Constants.Shooter.kMaxVelocityRadPerSecond,
                Constants.Shooter.kMaxAccelerationRadPerSecSquared)
            ),
            Constants.Shooter.homePosition
        );
        getController().setTolerance(Constants.Shooter.pivotTolerance);
        topShooterPIDController.setTolerance(Constants.Shooter.shooterTolerance);
        bottomShooterPIDController.setTolerance(Constants.Shooter.shooterTolerance);

        m_pivotMotor.setInverted(false);

        configureNeutralMode();

        enable();
    }

    // /**
    //  * Returns a command that will execute a quasistatic test in the given direction.
    //  *
    //  * @param direction The direction (forward or reverse) to run the test in
    //  */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.quasistatic(direction);
    // }

    // /**
    //  * Returns a command that will execute a dynamic test in the given direction.
    //  *
    //  * @param direction The direction (forward or reverse) to run the test in
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.dynamic(direction);
    // }

    public void goHome() {
        setGoal(Constants.Shooter.homePosition);
    }

    public void rampShooter(double topRPM, double bottomRPM) {
        isShooting = true;
        double topFeedforward = topShooterFeedforward.calculate(topRPM);
        double topPidOutput = topShooterPIDController.calculate(getShooterTopRPM(), topRPM);

        double bottomFeedforward = bottomShooterFeedforward.calculate(bottomRPM);
        double bottomPidOutput = bottomShooterPIDController.calculate(getShooterBottomRPM(), bottomRPM);

        m_shooterBottomMotor.setVoltage(bottomPidOutput + bottomFeedforward);
        m_shooterTopMotor.setVoltage(topPidOutput + topFeedforward);
    }

    public void tuneTest(double rpm) {
        double Feedforward = bottomShooterFeedforward.calculate(rpm);
        double PidOutput = bottomShooterPIDController.calculate(getShooterBottomRPM(), rpm);

        m_shooterBottomMotor.setVoltage(PidOutput + Feedforward);
    }

    public void pivot(TrapezoidProfile.State setpoint) {
        double pidOutput = getController().calculate(getPivotRadians(), setpoint.position);
        double feedforward = pivotFeedforward.calculate(setpoint.position, setpoint.velocity);

        m_pivotMotor.setVoltage(pidOutput + feedforward);
    }

    /**
     * @param range the robot's distance from the speaker given by the april tag
     * @return the rpm calculated by the interpolator
     */
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterInterpolator.kUseFlywheelAutoAimPolynomial) {
            return ShooterInterpolator.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterInterpolator.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    /**
     * @param range the robot's distance from the speaker given by the april tag
     * @return the pivot calculated by the interpolator
     */
    private double getPivotSetpointFromRegression(double range) {
        if (ShooterInterpolator.kUsePivotAutoAimPolynomial) {
            return ShooterInterpolator.kPivotAutoAimPolynomial.predict(range);
        } else {
            return ShooterInterpolator.kPivotAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    public Command applykS() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.Shooter.pivotkS),
            () -> m_pivotMotor.stopMotor()
        );
    }

    public Command applykG() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Math.signum(Math.sin(getMeasurement() + Math.PI / 2)) * Constants.Shooter.pivotkS + Math.cos(getMeasurement()) * Constants.Shooter.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }

    public Command applykV() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.Shooter.pivotkV + Math.signum(Math.sin(getMeasurement() + Math.PI / 2)) * Constants.Shooter.pivotkS + Math.cos(getMeasurement()) * Constants.Shooter.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }


    public boolean pivotAtSetpoint() {
        // return getController().atGoal();
        return Math.abs(getMeasurement() - getController().getGoal().position) <= Constants.Shooter.pivotTolerance * 3; // TODO get rid of multiplier
    }

    public boolean bottomShooterAtSetpoint() {
        return Math.abs(getShooterBottomRPM() - bottomShooterPIDController.getSetpoint()) <= Constants.Shooter.shooterTolerance * 1.5;
    }

    public boolean topShooterAtSetpoint() {
        // return topShooterPIDController.getSetpoint() >= 100 
        return Math.abs(getShooterTopRPM() - topShooterPIDController.getSetpoint()) <= Constants.Shooter.shooterTolerance * 1.5;
        // return topShooterPIDController.atSetpoint();
    }

    public double getCANCoder() {
        return m_cancoder.getAbsolutePosition().getValueAsDouble() * 360 - 90 + 71 - 5.7;
    }

    public double getCANCoderVelocityRadians() {
        return m_cancoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    
    // Just for shooter pivot
    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // System.out.println(setpoint.position + " " + setpoint.velocity);
        double feedforward = pivotFeedforward.calculate(setpoint.position, setpoint.velocity);
        // System.out.println(output + feedforward);
        m_pivotMotor.setVoltage(output + feedforward);
    }

    public void setPivot(double goal) {
        setGoal(goal);
    }

    @Override
    public double getMeasurement() {
        return getCANCoder() * Math.PI / 180 + 0.1 + 1.35 + 0.05;
        // return getPivotRadians();
    }

    private double getPivotVelocity() {
        return m_cancoder.getVelocity().getValueAsDouble() * 360 * Math.PI / 180;
    }

    public Command readyShootCommand(Supplier<Double> distanceSupplier/* , Supplier<Boolean> feed*/) {
        double pivotAngle = getPivotSetpointFromRegression(distanceSupplier.get());
        double rpm = getShooterSetpointFromRegression(distanceSupplier.get());

        
        // System.out.println(pivotAngle + ", " + rpm);
        // return Commands.run(() -> {
        //     System.out.println(getPivotSetpointFromRegression(distanceSupplier.get()) + ", " + getShooterSetpointFromRegression(distanceSupplier.get()));
        // });
        return Commands.runEnd(
            () -> {
                rampShooter(getShooterSetpointFromRegression(distanceSupplier.get()), getShooterSetpointFromRegression(distanceSupplier.get()));
                setPivot(getPivotSetpointFromRegression(distanceSupplier.get()));
            },
            () -> stopShooter()
        );
        // return new RampShooter(rpm, rpm, pivotAngle); // TODO fix

        // return new ParallelCommandGroup(
        //     new Pivot(
        //         // distanceSupplier,
        //         pivotAngle
        //     ),
        //     new RampShooter(
        //         // distanceSupplier,
        //         rpm
        //     )
        // );
    }

    public double getShooterBottomRPM() {
        return m_shooterBottomMotor.getVelocity().getValue() * 60;
    }

    public double getShooterTopRPM() {
        return m_shooterTopMotor.getVelocity().getValue() * 60;
    }

    /**
     * @return the angle of the shooter pivot in radians
     */
    private double getPivotDegrees() {
        return m_pivotMotor.getPosition().getValueAsDouble() * 12.375;
        // return Conversions.armRotationsToRadians(m_cancoder.getAbsolutePosition().getValue(), 1);
    }

    private double getPivotRadians() {
        return Units.degreesToRadians(getPivotDegrees());
        // return Conversions.armRotationsToRadians(m_cancoder.getAbsolutePosition().getValue(), 1);
    }

    public void stopPivot() {
        m_pivotMotor.stopMotor();
    }

    public void stopShooter() {
        m_shooterBottomMotor.stopMotor();
        m_shooterTopMotor.stopMotor();
        isShooting = false;
        isScoringAmp = false;
    }

    public boolean isReadyToShoot() {
        return topShooterAtSetpoint()
        && bottomShooterAtSetpoint()
        && pivotAtSetpoint()
        && isShooting;
    }

    public Command shooterReadyLEDCommand() {
        return RobotContainer.s_Led.flashUntilCommand(LEDColor.GREEN, 0.1, () -> !isReadyToShoot());
    }

    public void manualShooterPivot(Boolean pivotingUp) {
        // double speed = Constants.Shooter.manualShooterPivotSpeed;
        double voltage = Math.cos(getMeasurement()) * Constants.Shooter.pivotkG;
        if(pivotingUp == null) {
            m_pivotMotor.setVoltage(voltage);
        }
        else {
            if(!pivotingUp) {
                voltage -= (Constants.Shooter.pivotkS + Constants.Shooter.manualShooterPivotVoltage);
            } else {
                voltage += (Constants.Shooter.pivotkS + Constants.Shooter.manualShooterPivotVoltage);
            }
            m_pivotMotor.setVoltage(voltage);
        }
            
        
    }

    public void manualShoot() {
        m_shooterTopMotor.set(Constants.Shooter.topShooterSpeed);
        m_shooterBottomMotor.set(Constants.Shooter.bottomShooterSpeed);
    }

    public Command feedToShooter() {
        return new ParallelCommandGroup(
            new Feed(),
            new ForkCommand(Intake.Direction.TO_SHOOTER)
        ).alongWith(
            NoteVisualizer.shoot()
        );
    }

    public Command feedToTrigShooter() {
        return new ParallelCommandGroup(
            new ForkCommand(Intake.Direction.TO_SHOOTER)
        ).alongWith(
            NoteVisualizer.shoot()
        );
    }

    public Command feedToShooterAmp() {
        return new ParallelCommandGroup(
            new FeedAmp(),
            new AmpForkCommand()
        );
    }

    public Command feedToIntakeFromShooter() {
        return new ParallelCommandGroup(
            Commands.runEnd(
                () -> {
                    setPivot(Constants.Shooter.shooterIntakeAngle);
                    m_shooterTopMotor.set(Constants.Shooter.shooterIntakeSpeed);
                    m_shooterBottomMotor.set(Constants.Shooter.shooterIntakeSpeed);
                },
                () -> stopShooter()
            ),
            new FeedtoIntake(),
            new ForktoIntakeCommand()
        ).until(() -> RobotContainer.s_Intake.intakeChangeFromBrokenToUnbroken());
    }

    private void configureNeutralMode() {
        m_shooterBottomMotor.setNeutralMode(NeutralModeValue.Coast);
        m_shooterTopMotor.setNeutralMode(NeutralModeValue.Coast);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_pivotMotor.setPosition(0);
    }

    // /**
    //  * Returns a command that will execute a quasistatic test in the given direction.
    //  *
    //  * @param direction The direction (forward or reverse) to run the test in
    //  */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.quasistatic(direction);
    // }

    // /**
    //  * Returns a command that will execute a dynamic test in the given direction.
    //  *
    //  * @param direction The direction (forward or reverse) to run the test in
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.dynamic(direction);
    // }

    @Override
    public void periodic() {

        
        SmartDashboard.putNumber("shooter/pivot cancoder radians", getMeasurement());
        // SmartDashboard.putNumber("shooter/pivot cancoder", getCANCoder());
        // SmartDashboard.putNumber("shooter/pivot voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
        // if(isReadyToShoot()) System.out.println("shooter is ready");
        // m_shooterTopMotor.set(1);
        // m_shooterBottomMotor.set(1);

        if (m_enabled) {
            // this.disable();
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
            // System.out.println(m_controller.getSetpoint().position);
        } else {
            // m_pivotMotor.setVoltage(0.6);
            // System.out.println(m_pivotMotor.getMotorVoltage().getValue());
        }

        // SmartDashboard.putBoolean("shooter/top ready", topShooterAtSetpoint());
        // SmartDashboard.putBoolean("shooter/bottom ready", bottomShooterAtSetpoint());
        // SmartDashboard.putBoolean("shooter/pivot ready", pivotAtSetpoint());

        // m_pivotMotor.setVoltage(0.4); 
        SmartDashboard.putNumber("shooter/bottom rpm ", getShooterBottomRPM()); 
        SmartDashboard.putNumber("shooter/top rpm ", getShooterTopRPM());   
        // SmartDashboard.putNumber("shooter/pivot deg", getPivotDegrees());
        // SmartDashboard.putNumber("shooter/pivot rad", getPivotRadians());
        // SmartDashboard.putNumber("shooter/pivot velocity", getCANCoderVelocityRadians());

        // SmartDashboard.putBoolean("shooter/is ramped", isReadyToShoot());
        // SmartDashboard.putNumber("shooter/top setpoint", topShooterPIDController.getSetpoint());
        // SmartDashboard.putNumber("shooter/top error", topShooterPIDController.getPositionError());
    }   

}