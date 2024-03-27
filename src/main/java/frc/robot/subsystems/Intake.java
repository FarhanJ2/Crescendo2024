package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED.LEDColor;

public class Intake extends SubsystemBase {

    public static enum Direction {
        TO_SHOOTER,
        TO_AMP_ARM,
        TO_INTAKE // For arm to intake
    }

    private final TalonFX m_intakeMotor1 = new TalonFX(Constants.Intake.intakeMotor1ID, Constants.canivoreName);
    private final TalonFX m_intakeMotor2 = new TalonFX(Constants.Intake.intakeMotor2ID, Constants.canivoreName);
    private final TalonFX m_forkMotor = new TalonFX(Constants.Intake.forkMotorID, Constants.canivoreName);

    private final DigitalInput intakeBeam = new DigitalInput(Constants.Intake.beamBreakerIntake);
    // private final DigitalInput shooterBeam = new DigitalInput(Constants.Intake.beamBrakerShooter);
    private final DigitalInput armBeam = new DigitalInput(Constants.Intake.beamBreakerArm);

    private boolean prevBeamBroken = false;

    public Intake() {
        configureMotors();
    }

    public void intake() {
        // m_intakeMotor.setVoltage(Constants.Intake.intakeVoltage);
        m_forkMotor.setVoltage(1);
        m_intakeMotor1.set(-Constants.Intake.intakeSpeed);
        m_intakeMotor2.set(-Constants.Intake.intakeSpeed);
    }

    public void intakeReverse() {
        m_intakeMotor1.set(Constants.Intake.intakeSpeed);
        m_intakeMotor2.set(Constants.Intake.intakeSpeed);
    }

    public void runFork(Direction direction) {
        if (direction == Direction.TO_SHOOTER) {
            m_forkMotor.set(Constants.Intake.forkSpeed);
            m_intakeMotor1.set(-Constants.Intake.intakeFeedSpeed);
            m_intakeMotor2.set(-Constants.Intake.intakeFeedSpeed);
        } 
        else if (direction == Direction.TO_AMP_ARM) {
            m_forkMotor.set(-Constants.Intake.forkSpeed / 3);
            m_intakeMotor1.set(-Constants.Intake.intakeFeedSpeed / 3);
            m_intakeMotor2.set(-Constants.Intake.intakeFeedSpeed / 3);
        }
        else if (direction == Direction.TO_INTAKE) {
            m_forkMotor.set(Constants.Intake.forkSpeed);
            m_intakeMotor1.set(Constants.Intake.intakeFeedSpeed / 3.5);
            m_intakeMotor2.set(Constants.Intake.intakeFeedSpeed / 3.5);
        }
    }

    public void forkToShooterLow() {
        m_forkMotor.setVoltage(1);
    }

    public void runForkAmp() {
        m_forkMotor.set(Constants.Shooter.ampForkSpeed);
        m_intakeMotor1.set(-Constants.Shooter.ampForkSpeed);
        m_intakeMotor2.set(-Constants.Shooter.ampForkSpeed);
    }

    public void runForkToIntake() {
        m_forkMotor.set(Constants.Shooter.forkToIntakeSpeed);
        m_intakeMotor1.set(Constants.Intake.intakeSpeed / 2);
        m_intakeMotor2.set(Constants.Intake.intakeSpeed / 2);
    }

    public Command intakeLedCommand() {
        return RobotContainer.s_Led.flashCommand(LEDColor.GREEN, 0.1, 1);
    }

    private void configureMotors() {
        m_intakeMotor1.optimizeBusUtilization();
        m_intakeMotor2.optimizeBusUtilization();
        m_forkMotor.optimizeBusUtilization();

        m_intakeMotor1.setInverted(false);
        m_intakeMotor2.setInverted(true);
        m_forkMotor.setInverted(true);

        m_intakeMotor1.setNeutralMode(NeutralModeValue.Coast);
        m_intakeMotor2.setNeutralMode(NeutralModeValue.Coast);
        m_forkMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public boolean intakeChangeFromBrokenToUnbroken() {
        if(intakeBeamBroken()) {
            prevBeamBroken = true;
            return false;
        } else {
            if(prevBeamBroken) {
                prevBeamBroken = false;
                return true;
            }
            else {
                return false;
            }
        }
    }

    public boolean intakeBeamBroken() {
        return !intakeBeam.get();
    }

    public boolean armBeamBroken() {
        return !armBeam.get();
    }

    public void stop() {
        m_intakeMotor1.stopMotor();
        m_intakeMotor2.stopMotor();
        m_forkMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake/intake beam", intakeBeamBroken());
        SmartDashboard.putBoolean("intake/arm beam", armBeamBroken());
    }
}
