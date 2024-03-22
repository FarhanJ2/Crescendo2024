package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.commands.AmpArm.ArmHandoff;
import frc.robot.commands.intake.ForkCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Intake;

public class AmpArm extends ProfiledPIDSubsystem {

    public static enum Position {
        HOME(Constants.AmpArm.homePosition),
        DANGLE(Constants.AmpArm.danglePosition),
        HANDOFF(Constants.AmpArm.handoffPosition),
        AMP_SLAM(Constants.AmpArm.ampSlamPosition),
        AMP_SHOOT(Constants.AmpArm.ampShootPosition),
        CLIMB_POSITION(Constants.AmpArm.climbIdlePosition),
        TRAP(Constants.AmpArm.trapPosition);

        private double rotations;

        private Position(double rotations) {
            this.rotations = rotations;
        }

        private double getRotations() {
            return rotations;
        }
    }
    // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    // private final MutableMeasure<Angle> m_angle = mutable(Radians.of(0));
    // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    // private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

    // private final Measure<Velocity<Voltage>> m_desiredRampRate = Volts.of(0.075).per(Seconds.of(1));
    // private final Measure<Voltage> m_desiredStepVoltage = Volts.of(0.6);

    private final TalonFX m_pivotMotor = new TalonFX(Constants.AmpArm.pivotMotorID, Constants.canivoreName);
    private final TalonFX m_shootMotor = new TalonFX(Constants.AmpArm.shootMotorID, Constants.canivoreName);

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
    //             log.motor("pivot-arm")
    //                 .voltage(
    //                     m_appliedVoltage.mut_replace(
    //                         m_pivotMotor.get() * RobotController.getBatteryVoltage(), Volts))
    //                 //.angularPosition(m_angle.mut_replace(m_pivotMotor.getPosition().getValue() / 18.8888888888888, Rotations))
    //                 .angularPosition(m_angle.mut_replace(getMeasurement(), Radians))
    //                 // .angularVelocity(
    //                 //     m_velocity.mut_replace(m_pivotMotor.getVelocity().getValue() / 18.8888888888888, RotationsPerSecond));
    //                 .angularVelocity(
    //                     m_velocity.mut_replace(getCANCoderVelocityRadians(), RadiansPerSecond));
    //           },
    //           // Tell SysId to make generated commands require this subsystem, suffix test state in
    //           // WPILog with this subsystem's name ("shooter")
    //           this));

    private final CANcoder m_cancoder = new CANcoder(Constants.AmpArm.canCoderID, Constants.canivoreName);

    private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.AmpArm.pivotkS, 
          Constants.AmpArm.pivotkG,
          Constants.AmpArm.pivotkV
    );
    
    public AmpArm() {
        super(
            new ProfiledPIDController(
                Constants.AmpArm.pivotkP,
                Constants.AmpArm.pivotkI,
                Constants.AmpArm.pivotkD,
                new TrapezoidProfile.Constraints(
                    Constants.AmpArm.kMaxVelocityRadPerSecond,
                    Constants.AmpArm.kMaxAccelerationRadPerSecSquared)),
            Constants.AmpArm.homePosition);

        getController().setTolerance(Constants.AmpArm.pivotTolerance);
        getController().setIZone(Constants.AmpArm.integratorZone);
        getController().enableContinuousInput(0, Math.PI * 2);
        
        enable();
        configureMotors();
    }

    // public Command getHomeCommand() {
    //     // return new InstantCommand(() -> setGoal(Position.HOME.getRotations()));
    //     return new ParallelDeadlineGroup(
    //         new WaitUntilCommand(m_limitSwitch::get)
    //     ).andThen(
    //         new InstantCommand(
    //             () -> m_cancoder.setPosition(0)
    //         )
    //     );
    // }

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
    
    public Command applykS() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.AmpArm.pivotkS),
            () -> m_pivotMotor.stopMotor()
        );
    }

    public Command applykG() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(/*Math.signum(Math.sin(getMeasurement() + Math.PI / 2)) * Constants.AmpArm.pivotkS + */Math.cos(getMeasurement()) * Constants.AmpArm.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }

    public Command applykV() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.AmpArm.pivotkV + /*Math.signum(Math.sin(getMeasurement() + Math.PI / 2)) * Constants.AmpArm.pivotkS + */Math.cos(getMeasurement()) * Constants.AmpArm.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }

    public Command getAmpSlamCommand() {
        return new InstantCommand(() -> {
            setGoal(Position.AMP_SLAM.getRotations());
            this.enable();
        }, this);
    }

    public Command getClimbPosition() {
        return new InstantCommand(() -> {
            setGoal(Position.CLIMB_POSITION.getRotations());
            this.enable();
        }, this);
    }

    public Command getAmpDangleCommand() {
        return new InstantCommand(() -> {
            setGoal(Position.DANGLE.getRotations());
            this.enable();
        }, this);
    }

    public Command getAmpShootCommand() {
        return new InstantCommand(() -> {
            setGoal(Position.AMP_SHOOT.getRotations());
            this.enable();
        }, this);
    }

    public Command getTrapCommand() {
        return new InstantCommand(() -> {
            setGoal(Position.TRAP.getRotations());
            this.enable();
        }, this);
    }

    public Command getHomeCommand() {
        return new InstantCommand(() -> {
            setGoal(Position.HOME.getRotations());
            this.enable(); 
        }, this);
    }

    public Command getHandoffCommand() {
        return new InstantCommand(() -> {
            setGoal(Position.HANDOFF.getRotations());
            this.enable();
        }, this);
    }

    public double getCANCoderPositionDegrees() {
        return ((m_cancoder.getAbsolutePosition().getValueAsDouble() * 360 + 360) - 63) % 360;
    }

    public double getCANCoderPositionRadians() {
        return m_cancoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getCANCoderVelocityDegrees() {
        return m_cancoder.getVelocity().getValueAsDouble() * 360;
    }

    public double getCANCoderVelocityRadians() {
        return m_cancoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }


    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // System.out.println(setpoint.position + " " + setpoint.velocity);
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_pivotMotor.setVoltage(output + feedforward);
    }

    public void manualArmPivot(Boolean pivotingUp) {
        // double voltage = 0.35;
        // if(pivotingUp == null) {
        //     m_pivotMotor.setVoltage(voltage);
        // }
        // else {
        //     if(!pivotingUp) {
        //         voltage -= voltage / 2;
        //     } else {
        //         voltage += voltage / 2;
        //     }
        //     m_pivotMotor.setVoltage(voltage);
        // }
        if(pivotingUp == null) {
            m_pivotMotor.setVoltage(Math.cos(getMeasurement()) * Constants.AmpArm.pivotkG);
        }
        else if(pivotingUp) {
            m_pivotMotor.setVoltage(0.5 + Math.cos(getMeasurement()) * Constants.AmpArm.pivotkG);
        }
        else if(!pivotingUp) {
            m_pivotMotor.setVoltage(-0.5 + Math.cos(getMeasurement()) * Constants.AmpArm.pivotkG);

        }
    }

    public void stopArm() {
        m_pivotMotor.stopMotor();
    }
  
    public void shoot() {
        m_shootMotor.set(Constants.AmpArm.shootSpeed);
    }

    public void armHandoff() {
        m_shootMotor.set(Constants.AmpArm.handoffSpeed);
    }

    public void stopShooter() {
        m_shootMotor.stopMotor();
    }

    public Command feedToArm() {
        return new ParallelDeadlineGroup(
            new ArmHandoff(true),
            new ForkCommand(Intake.Direction.TO_AMP_ARM)
        );
    }

    public Command armToIntake() {
        return new ParallelDeadlineGroup(
            new ArmHandoff(false),
            new ForkCommand(Intake.Direction.TO_INTAKE)
        );
    }
    

    private void configureMotors() {
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_shootMotor.setNeutralMode(NeutralModeValue.Coast);
        m_pivotMotor.setPosition(-90 / 14.7);
    }

    private double getPivotDegrees() {
        return m_pivotMotor.getPosition().getValueAsDouble() /*/ 2048*/ * 14.7;
    }

    private double getPivotRadians() {
        return Units.degreesToRadians(getPivotDegrees());
    }

    // TODO need to change this
    @Override
    public double getMeasurement() {
        // return Conversions.armRotationsToRadians(m_cancoder.getAbsolutePosition().getValue(), 1) + Constants.AmpArm.armOffset;
        // return 0;
        // return getPivotRadians();
        // return m_pivotMotor.getPosition().getValueAsDouble();
        double preConversion = convert360To180(((getCANCoderPositionDegrees() /*+ 110*/)) % 360) * Math.PI / 180;
        return preConversion /*  - 3.05*/ /* - (Math.PI / 2) */ /* + 6.35*/ + 3.425 - Math.PI;
    }
    
    private double convert360To180(double angle) {
        return (angle + 180) % 360 - 180;
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("arm/Voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
        
        if (m_enabled) {
            // this.disable();
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
            // System.out.println(m_controller.getSetpoint().position);
        }

        // SmartDashboard.putNumber("testNum", 0);
        // System.out.println(SmartDashboard.getNumber("testNum", 0));
        
        SmartDashboard.putNumber("arm/motor rotations", m_pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor degrees", getPivotDegrees());
        SmartDashboard.putNumber("arm/cancoder radians", getMeasurement());

        SmartDashboard.putNumber("arm/cancoder degrees", getCANCoderPositionDegrees());

        SmartDashboard.putNumber("arm/velocity", getCANCoderVelocityRadians());

        SmartDashboard.putNumber("arm/pure can position", getCANCoderPositionRadians());
    }
}
