package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Elevator extends ProfiledPIDSubsystem {

    public static enum Level {
        CLIMB(Constants.Elevator.climbRotations),
        TRAP(Constants.Elevator.trapRotations),
        AMP(Constants.Elevator.ampRotations),
        HOME(Constants.Elevator.homeRotations);

        private double rotations;

        private Level(double rotations) {
            this.rotations = rotations; 
        }

        public double getRotations() {
            return rotations;
        }
    }

    private final TalonFX m_leftMotor = new TalonFX(Constants.Elevator.leftMotorID);
    private final TalonFX m_rightMotor = new TalonFX(Constants.Elevator.rightMotorID);



    // private final DigitalInput m_upperLimitSwitch = new DigitalInput(Constants.Elevator.limitSwitchUpperChannel);
    private final DigitalInput m_lowerLimitSwitch = new DigitalInput(Constants.Elevator.limitSwitchLowerChannel);

    private final CANcoder m_cancoder = new CANcoder(Constants.Elevator.canCoderID);

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private final Measure<Velocity<Voltage>> m_desiredRampRate = Volts.of(0.25).per(Seconds.of(1));
    private final Measure<Voltage> m_desiredStepVoltage = Volts.of(3);

    private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(m_desiredRampRate, m_desiredStepVoltage, null),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                m_leftMotor.setVoltage(volts.in(Volts));
                m_rightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("elevator")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    //.angularPosition(m_angle.mut_replace(m_pivotMotor.getPosition().getValue() / 18.8888888888888, Rotations))
                    .angularPosition(m_angle.mut_replace(getMeasurement(), Rotations))
                    // .angularVelocity(
                    //     m_velocity.mut_replace(m_pivotMotor.getVelocity().getValue() / 18.8888888888888, RotationsPerSecond));
                    .angularVelocity(
                        m_velocity.mut_replace(m_cancoder.getVelocity().getValueAsDouble(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

    private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.Elevator.kS, 
          Constants.Elevator.kG,
          Constants.Elevator.kV
    );
    
    public Elevator() {
        super(
            new ProfiledPIDController(
                Constants.Elevator.kP,
                Constants.Elevator.kI,
                Constants.Elevator.kD,
                new TrapezoidProfile.Constraints(
                    Constants.Elevator.kMaxVelocityPerSecond,
                    Constants.Elevator.kMaxAccelerationPerSecSquared)),
            0);

        getController().setTolerance(Constants.Elevator.tolerance);
        m_cancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive)));

        zeroCancoder();
        
        configureMotors();

        enable();
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
    

    public Command applykS() {
        return Commands.runEnd(
            () -> {
                m_rightMotor.setVoltage(-Constants.Elevator.kS);
                m_leftMotor.setVoltage(-Constants.Elevator.kS);
            },
            () -> {
                m_rightMotor.stopMotor();
                m_leftMotor.stopMotor();
            }
        );
    }

    // TODO get what limit is when it's pressed (true or false)
    // public boolean upperLimitPressed() {
    //     return m_upperLimitSwitch.get();
    // }

    // public boolean lowerLimitPressed() {
    //     return m_lowerLimitSwitch.get();
    // }

    // public Command getHomeCommand() {
    //     // return new InstantCommand(() -> setGoal(Level.HOME.getRotations()));
    //     // TODO  dont know if this works
    //     return new ParallelDeadlineGroup(
    //         new WaitUntilCommand(m_lowerLimitSwitch::get)
    //     ).andThen(
    //         new InstantCommand(
    //             () -> m_cancoder.setPosition(0)
    //         )
    //     );
    // }

    public Command getHomeCommand() {
        //TODO add limit switch
        return new InstantCommand(() -> setGoal(Level.HOME.getRotations()));
    }

    public Command getAmpCommand() {
        return new InstantCommand(() -> setGoal(Level.AMP.getRotations()));
    }

    public Command getClimbCommand() {
        return new InstantCommand(() -> setGoal(Level.CLIMB.getRotations()));
    }

    public Command getTrapCommand() {
        return new InstantCommand(() -> {
            setGoal(Level.TRAP.getRotations());
            System.out.println("running elevator");
        });
    }

    public double getEncoderRotations() {
        return (m_cancoder.getPosition().getValue());
    }

    // public double getCANCoder() {
    //     return m_cancoder.getAbsolutePosition().getValueAsDouble() * 360;
    // }

    // public void moveElevator() {
    //     m_motorOne.setVoltage(getMeasurement());
      
    // }

    public void moveElevator(boolean isDown) {
        double speed = Constants.Elevator.manualElevatorSpeed;
        if(isDown) {
            // if(getEncoderRotations() < 0.05) {
            //     stopElevator();
            //     return;
            // }
            if(!m_lowerLimitSwitch.get()) {
                stopElevator();
                zeroCancoder();
                return;
            }
            
        }
        else {
            if(getEncoderRotations() > 2.8) {
                stopElevator();
                return;
            }
            speed = -speed;
        }
        m_leftMotor.set(speed);
        m_rightMotor.set(speed);
    }

    private void zeroCancoder() {
        m_cancoder.setPosition(0);
    }

    public void stopElevator() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    private void configureMotors() {
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);

        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {

        // if (!m_lowerLimitSwitch.get() || getEncoderRotations() >= 2.8) return;
        // System.out.println("position: " + setpoint.position);

        double feedforward = m_feedforward.calculate(setpoint.velocity);

        // System.out.println(feedforward);

        m_leftMotor.setVoltage(-(output + feedforward));
        m_rightMotor.setVoltage(-(output + feedforward));
    }

    @Override
    public double getMeasurement() {
        return getEncoderRotations();
    }

    @Override
    public void periodic() {

        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
        }

        SmartDashboard.putNumber("elevator/motor voltage", m_leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/using battery voltage", m_leftMotor.get() * RobotController.getBatteryVoltage());


        // SmartDashboard.putNumber("elevator/cancoder", getCANCoder()); // 36 max top
        SmartDashboard.putNumber("elevator/cancoder", m_cancoder.getPosition().getValue());
        SmartDashboard.putNumber("elevator/actual measurement", getMeasurement());
        SmartDashboard.putNumber("elevator/rotations", getEncoderRotations());
        SmartDashboard.putBoolean("elevator/lower limit", m_lowerLimitSwitch.get());
        SmartDashboard.putNumber("elevator/velocity", m_cancoder.getVelocity().getValueAsDouble());
    }
}
