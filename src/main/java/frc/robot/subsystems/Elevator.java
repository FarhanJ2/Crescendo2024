package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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

    private final TalonFX m_leftMotor = new TalonFX(Constants.Elevator.leftMotorID);
    private final TalonFX m_rightMotor = new TalonFX(Constants.Elevator.rightMotorID);

    // private final DigitalInput m_upperLimitSwitch = new DigitalInput(Constants.Elevator.limitSwitchUpperChannel);
    // private final DigitalInput m_lowerLimitSwitch = new DigitalInput(Constants.Elevator.limitSwitchLowerChannel);

    private final CANcoder m_cancoder = new CANcoder(Constants.Elevator.canCoderID);

    private double initialRotations;

    private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.AmpArm.pivotkS, 
          Constants.AmpArm.pivotkG,
          Constants.AmpArm.pivotkV
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

        initialRotations = getEncoderRotations();
        
        configureMotors();
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
        return new InstantCommand(() -> setGoal(Level.TRAP.getRotations()));
    }

    public double getEncoderRotations() {
        return (m_cancoder.getPosition().getValue() - initialRotations) / 360;
    }

    // public double getCANCoder() {
    //     return m_cancoder.getAbsolutePosition().getValueAsDouble() * 360;
    // }

    // public void moveElevator() {
    //     m_motorOne.setVoltage(getMeasurement());
      
    // }

    public void moveElevator(boolean isUp) {
        double speed = Constants.Elevator.manualElevatorSpeed;
        if(!isUp) {
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
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_leftMotor.setVoltage(output + feedforward);
        m_rightMotor.setVoltage(output + feedforward);
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

        // SmartDashboard.putNumber("elevator/cancoder", getCANCoder()); // 36 max top
        SmartDashboard.putNumber("elevator/cancoder", getEncoderRotations());
    }
}
