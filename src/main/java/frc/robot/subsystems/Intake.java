package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public static enum Direction {
        TO_SHOOTER,
        TO_AMP_ARM,
        TO_INTAKE // For arm to intake
    }

    private final TalonFX m_intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);
    private final TalonFX m_forkMotor = new TalonFX(Constants.Intake.forkMotorID);

    private final DigitalInput intakeBeam = new DigitalInput(Constants.Intake.beamBreakerIntake);
    // private final DigitalInput shooterBeam = new DigitalInput(Constants.Intake.beamBrakerShooter);
    private final DigitalInput armBeam = new DigitalInput(Constants.Intake.beamBreakerArm);

    public Intake() {
        configureMotors();
    }

    public void intake() {
        // m_intakeMotor.setVoltage(Constants.Intake.intakeVoltage);
        m_intakeMotor.set(-Constants.Intake.intakeSpeed);
    }

    public void intakeReverse() {
        m_intakeMotor.set(Constants.Intake.intakeSpeed);
    }

    public void runFork(Direction direction) {
        if (direction == Direction.TO_SHOOTER) {
            m_forkMotor.set(Constants.Intake.forkSpeed);
            m_intakeMotor.set(-Constants.Intake.intakeFeedSpeed);
        } else if (direction == Direction.TO_AMP_ARM) {
            m_forkMotor.set(-Constants.Intake.forkSpeed / 3.5);
            m_intakeMotor.set(-Constants.Intake.intakeFeedSpeed / 3.5);
        }
        else if (direction == Direction.TO_INTAKE) {
            m_forkMotor.set(Constants.Intake.forkSpeed);
            m_intakeMotor.set(Constants.Intake.intakeFeedSpeed / 3.5);
        }
    }

    private void configureMotors() {
        m_intakeMotor.setInverted(false);
        m_forkMotor.setInverted(true);

        m_intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        m_forkMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public boolean intakeBeamBroken() {
        return !intakeBeam.get();
    }

    public boolean armBeamBroken() {
        return !armBeam.get();
    }

    public void stop() {
        m_intakeMotor.stopMotor();
        m_forkMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake/intake beam", intakeBeamBroken());
        SmartDashboard.putBoolean("intake/arm beam", armBeamBroken());
    }
}
