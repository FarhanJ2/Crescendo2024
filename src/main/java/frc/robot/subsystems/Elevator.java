package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
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

    private final TalonFX m_leftMotor = new TalonFX(Constants.Elevator.leftMotorID, Constants.canivoreName);
    private final TalonFX m_rightMotor = new TalonFX(Constants.Elevator.rightMotorID, Constants.canivoreName);

    private final DigitalInput m_lowerLimitSwitch = new DigitalInput(Constants.Elevator.limitSwitchLowerChannel);

    private final CANcoder m_cancoder = new CANcoder(Constants.Elevator.canCoderID, Constants.canivoreName);

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
        m_cancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        zeroCancoder();
        configureMotors();

        enable();
    }

    public Command getHomeCommand() {
        return Commands.runOnce(this::disable)
        .andThen(
            Commands.run(
                () -> moveElevator(true),
                this
        ).until(this::limitPressed)
        .andThen(Commands.runOnce(this::stopElevator)));
    }

    public Command getAmpCommand() {
        return new InstantCommand(() -> {
            setGoal(Level.AMP.getRotations());
            enable();
        });
    }

    public Command getClimbCommand() {
        return new InstantCommand(() -> {
            setGoal(Level.CLIMB.getRotations());
            enable();
        });
    }

    public Command getTrapCommand() {
        return new InstantCommand(() -> {
            setGoal(Level.TRAP.getRotations());
            enable();
        });
    }

    public double getEncoderRotations() {
        return (m_cancoder.getPosition().getValue());
    }

    // For manual control
    public void moveElevator(boolean isDown) {
        double speed = Constants.Elevator.manualElevatorSpeed;
        if(isDown) {
            if(limitPressed()) {
                stopElevator();
                return;
            }
            
        }
        else {
            if(elevatorAtMax()) {
                stopElevator();
                return;
            }
            speed = -speed;
        }
        m_leftMotor.set(speed);
        m_rightMotor.set(speed);
    }

    private boolean elevatorAtMax() {
        return getMeasurement() >= Constants.Elevator.maxRotations;
    }

    private void zeroCancoder() {
        m_cancoder.setPosition(0);
    }

    public void stopElevator() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    private void configureMotors() {
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {

        double feedforward = m_feedforward.calculate(setpoint.velocity);
        double volts = -(output + feedforward);

        if ((limitPressed() && volts >= 0) || (elevatorAtMax() && volts <= 0)) {
            stopElevator();
            return;
        }

        m_leftMotor.setVoltage(volts);
        m_rightMotor.setVoltage(volts);
    }

    public boolean limitPressed() {
        return !m_lowerLimitSwitch.get();
    }

    @Override
    public double getMeasurement() {
        return getEncoderRotations();
    }

    /* TUNING */
    public Command applykG() {
        return Commands.runEnd(
            () -> {
                m_rightMotor.setVoltage(-Constants.Elevator.kG);
                m_leftMotor.setVoltage(-Constants.Elevator.kG);
            },
            () -> {
                m_rightMotor.stopMotor();
                m_leftMotor.stopMotor();
            }
        );
    }

    public Command applykV() {
        return Commands.runEnd(
            () -> {
                m_rightMotor.setVoltage(-Constants.Elevator.kG - Constants.Elevator.kV);
                m_leftMotor.setVoltage(-Constants.Elevator.kG - Constants.Elevator.kV);
            },
            () -> {
                m_rightMotor.stopMotor();
                m_leftMotor.stopMotor();
            }
        );
    }

    @Override
    public void periodic() {

        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
        }

        SmartDashboard.putNumber("elevator/motor voltage", m_leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rotations", getMeasurement());
        SmartDashboard.putBoolean("elevator/lower limit", m_lowerLimitSwitch.get());
    }
}
