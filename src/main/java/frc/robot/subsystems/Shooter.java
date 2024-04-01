package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.util.NoteVisualizer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.feeder.FeedAmp;
import frc.robot.commands.feeder.FeedtoIntake;
import frc.robot.commands.intake.AmpForkCommand;
import frc.robot.commands.intake.ForkCommand;
import frc.robot.commands.intake.ForktoIntakeCommand;
import frc.robot.regression.ShooterInterpolator;
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

    // Shooter feedback
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
    
    // Shooter feedforward
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
    
    // Pivot feedforward
    private final ArmFeedforward pivotFeedforward =
        new ArmFeedforward(
            Constants.Shooter.pivotkS, 
            Constants.Shooter.pivotkG,
            Constants.Shooter.pivotkV
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

        // getController().setTolerance(Constants.Shooter.pivotTolerance);
        topShooterPIDController.setTolerance(Constants.Shooter.shooterTolerance);
        bottomShooterPIDController.setTolerance(Constants.Shooter.shooterTolerance);

        m_pivotMotor.setInverted(false);

        configureNeutralMode();

        enable();
        m_shooterBottomMotor.setVoltage(trigTargetAngle);
    }

    public void goHome() {
        setGoal(Constants.Shooter.homePosition);
    }

    public void runShooterIdle() {
        m_shooterBottomMotor.set(Constants.Shooter.idleSpeed);
        m_shooterTopMotor.set(Constants.Shooter.idleSpeed);
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

    public Command feedToShooter() {
        return new ParallelCommandGroup(
            new Feed(),
            new ForkCommand(Intake.Direction.TO_SHOOTER))
        .alongWith(
            NoteVisualizer.shoot()
        );
    }

    public Command feedToTrigShooter() {
        return new ParallelCommandGroup(
            new ForkCommand(Intake.Direction.TO_SHOOTER))
        .alongWith(
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

    // Just for shooter pivot
    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = pivotFeedforward.calculate(setpoint.position, setpoint.velocity);
        m_pivotMotor.setVoltage(output + feedforward);

        SmartDashboard.putNumber("shooter/feedforward", feedforward);
        SmartDashboard.putNumber("shooter/setpoint position", setpoint.position);
        SmartDashboard.putNumber("shooter/setpoint velocity", setpoint.velocity);


    }

    public void setPivot(double goal) {
        setGoal(goal);
    }

    public void manualShooterPivot(Boolean pivotingUp) {
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

    public boolean pivotAtSetpoint() {
        return Math.abs(getMeasurement() - getController().getGoal().position) <= Constants.Shooter.pivotTolerance * 1.5;
    }

    public boolean bottomShooterAtSetpoint() {
        return Math.abs(getShooterBottomRPM() - bottomShooterPIDController.getSetpoint()) <= Constants.Shooter.shooterTolerance * 1.5;
    }

    public boolean topShooterAtSetpoint() {
        return Math.abs(getShooterTopRPM() - topShooterPIDController.getSetpoint()) <= Constants.Shooter.shooterTolerance * 1.5;
    }

    public double getCANCoder() {
        return m_cancoder.getAbsolutePosition().getValueAsDouble() * 360 - 24.7;
    }

    public double getCANCoderVelocityRadians() {
        return m_cancoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    @Override
    public double getMeasurement() {
        return getCANCoder() * Math.PI / 180 + 3.05;
    }

    public double getShooterBottomRPM() {
        return m_shooterBottomMotor.getVelocity().getValue() * 60;
    }

    public double getShooterTopRPM() {
        return m_shooterTopMotor.getVelocity().getValue() * 60;
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

    // Maximum rpm is 3500
    public double getTrigShotRPM(double distance) {
        return Math.min(3000, distance * 500);
    }

    public void manualShoot() {
        m_shooterTopMotor.set(Constants.Shooter.topShooterSpeed);
        m_shooterBottomMotor.set(Constants.Shooter.bottomShooterSpeed);
    }

    private void configureNeutralMode() {
        m_shooterBottomMotor.setNeutralMode(NeutralModeValue.Coast);
        m_shooterTopMotor.setNeutralMode(NeutralModeValue.Coast);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_pivotMotor.setPosition(0);
    }

    /* REGRESSIONS (NOT USED) */
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

    // Ramps shooter using regression (not used)
    public Command readyShootCommand(Supplier<Double> distanceSupplier) {
        return Commands.runEnd(
            () -> {
                rampShooter(getShooterSetpointFromRegression(distanceSupplier.get()), getShooterSetpointFromRegression(distanceSupplier.get()));
                setPivot(getPivotSetpointFromRegression(distanceSupplier.get()));
            },
            () -> stopShooter()
        );
    }

    /* TUNING */
    public Command applykS() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.Shooter.pivotkS),
            () -> m_pivotMotor.stopMotor()
        );
    }
    public Command applykG() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Math.cos(getMeasurement()) * Constants.Shooter.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }
    public Command applykV() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.Shooter.pivotkV + Math.cos(getMeasurement()) * Constants.Shooter.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }

    private double getMotorVelocity() {
        return (m_pivotMotor.getVelocity().getValueAsDouble()*2*Math.PI) / ((15)*(60/28)*(64/18));
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
        }

        SmartDashboard.putNumber("shooter/goal", getController().getGoal().position);

        SmartDashboard.putNumber("shooter/bottom rpm ", getShooterBottomRPM()); 
        SmartDashboard.putNumber("shooter/top rpm ", getShooterTopRPM());
        SmartDashboard.putNumber("shooter/pivot angle", getMeasurement());
        SmartDashboard.putNumber("shooter/can velocity", getCANCoderVelocityRadians());
        SmartDashboard.putNumber("shooter/motor velocity", getMotorVelocity());
        SmartDashboard.putNumber("shooter/pivot voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
    }   

}