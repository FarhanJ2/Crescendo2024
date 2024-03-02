package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.intake.ForkCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.ManualShot;
import frc.robot.commands.shooter.Pivot;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.regression.ShooterInterpolator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Direction;
import frc.robot.subsystems.LED.LEDColor;
import frc.team254.lib.util.InterpolatingDouble;

public class Shooter extends ProfiledPIDSubsystem {
    private final TalonFX m_shooterBottomMotor = new TalonFX(Constants.Shooter.bottomShooterMotorID);
    private final TalonFX m_shooterTopMotor = new TalonFX(Constants.Shooter.topShooterMotorID);
    private final TalonFX m_pivotMotor = new TalonFX(Constants.Shooter.pivotMotorID);

    private final CANcoder m_cancoder = new CANcoder(Constants.Shooter.canCoderID);

    public boolean isShooting = false;

    /* Syd ID */
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private final Measure<Velocity<Voltage>> m_desiredRampRate = Volts.of(0.09).per(Seconds.of(1));
    private final Measure<Voltage> m_desiredStepVoltage = Volts.of(0.5);

    //Create a new SysId routine for characterizing the shooter.
    

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

    // private boolean isReadyToShoot;

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

        m_pivotMotor.setInverted(true);

        // isReadyToShoot = false;

        configureNeutralMode();

        enable();
    }

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
        // System.out.println(PidOutput + Feedforward);
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

    public boolean pivotAtSetpoint() {
        // return getController().atGoal();
        return Math.abs(getMeasurement() - getController().getGoal().position) <= Constants.Shooter.pivotTolerance * 5;
    }

    public boolean bottomShooterAtSetpoint() {
        return Math.abs(getShooterBottomRPM() - bottomShooterPIDController.getSetpoint()) <= Constants.Shooter.shooterTolerance * 3;
    }

    public boolean topShooterAtSetpoint() {
        // return topShooterPIDController.getSetpoint() >= 100 
        return Math.abs(getShooterTopRPM() - topShooterPIDController.getSetpoint()) <= Constants.Shooter.shooterTolerance * 3;
        // return topShooterPIDController.atSetpoint();
    }

    public double getCANCoder() {
        return m_cancoder.getAbsolutePosition().getValueAsDouble() * 360 - 90;
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
        return getCANCoder() * Math.PI / 180;
        // return getPivotRadians();
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
    }

    public boolean isReadyToShoot() {
        return topShooterAtSetpoint()
        && bottomShooterAtSetpoint()
        && pivotAtSetpoint();
        // && isShooting;
    }

    public Command shooterReadyLEDCommand() {
        return RobotContainer.s_Led.flashUntilCommand(LEDColor.GREEN, 0.1, () -> !isReadyToShoot());
    }

    public void manualShooterPivot(Boolean pivotingUp) {
        double speed = Constants.Shooter.manualShooterPivotSpeed;
        double voltage = 0.22;
        if(pivotingUp == null) {
            m_pivotMotor.setVoltage(voltage);
        }
        else {
            if(!pivotingUp) {
                voltage -= voltage / 2;
            } else {
                voltage += voltage / 2;
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
            // new IntakeCommand() TODO fix
        );
        //TODO make it so that it stops when not ready to shoot
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
        SmartDashboard.putNumber("shooter/pivot cancoder", getCANCoder());
        SmartDashboard.putNumber("shooter/pivot voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
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

        SmartDashboard.putBoolean("shooter/top ready", topShooterAtSetpoint());
        SmartDashboard.putBoolean("shooter/bottom ready", bottomShooterAtSetpoint());
        SmartDashboard.putBoolean("shooter/pivot ready", pivotAtSetpoint());

        // m_pivotMotor.setVoltage(0.4); 
        SmartDashboard.putNumber("shooter/bottom rpm ", getShooterBottomRPM()); 
        SmartDashboard.putNumber("shooter/top rpm ", getShooterTopRPM());   
        SmartDashboard.putNumber("shooter/pivot deg", getPivotDegrees());
        SmartDashboard.putNumber("shooter/pivot rad", getPivotRadians());

        SmartDashboard.putBoolean("shooter/is ramped", isReadyToShoot());
        SmartDashboard.putNumber("shooter/top setpoint", topShooterPIDController.getSetpoint());
        SmartDashboard.putNumber("shooter/top error", topShooterPIDController.getPositionError());
    }   

}