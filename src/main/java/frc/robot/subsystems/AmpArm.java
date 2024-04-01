package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.commands.AmpArm.ArmHandoff;
import frc.robot.commands.intake.ForkCommand;

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

    public static enum ArmStatus {
        NOTHING,
        INTAKING
    }

    private final TalonFX m_pivotMotor = new TalonFX(Constants.AmpArm.pivotMotorID, Constants.canivoreName);
    private final TalonFX m_shootMotor = new TalonFX(Constants.AmpArm.shootMotorID, Constants.canivoreName);

    private final CANcoder m_cancoder = new CANcoder(Constants.AmpArm.canCoderID, Constants.canivoreName);

    public ArmStatus status = ArmStatus.NOTHING; 

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

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_pivotMotor.setVoltage(output + feedforward);
    }

    public void manualArmPivot(Boolean pivotingUp) {
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
  
    public void shoot() {
        m_shootMotor.set(Constants.AmpArm.shootSpeed);
    }

    public void armHandoff() {
        m_shootMotor.set(Constants.AmpArm.handoffSpeed);
    }

    public void stopShooter() {
        m_shootMotor.stopMotor();
    }

    public void stopArm() {
        m_pivotMotor.stopMotor();
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

    public boolean inHandoffPosition() {
        return Math.abs(getMeasurement() - Position.HANDOFF.getRotations()) <= Constants.AmpArm.pivotTolerance * 5;
    }

    private void configureMotors() {
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_shootMotor.setNeutralMode(NeutralModeValue.Coast);
        m_pivotMotor.setPosition(-90 / 14.7);

        m_shootMotor.optimizeBusUtilization();
    }

    private double convert360To180(double angle) {
        return (angle + 180) % 360 - 180;
    }

    @Override
    public double getMeasurement() {
        double preConversion = convert360To180(((getCANCoderPositionDegrees())) % 360) * Math.PI / 180;
        return preConversion + 3.425 - Math.PI;
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

    /* TUNING */
    public Command applykS() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.AmpArm.pivotkS),
            () -> m_pivotMotor.stopMotor()
        );
    }

    public Command applykG() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Math.cos(getMeasurement()) * Constants.AmpArm.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }

    public Command applykV() {
        return Commands.runEnd(
            () -> m_pivotMotor.setVoltage(Constants.AmpArm.pivotkV + Math.cos(getMeasurement()) * Constants.AmpArm.pivotkG),
            () -> m_pivotMotor.stopMotor()
        );
    }

    @Override
    public void periodic() {
        
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
        }

        SmartDashboard.putNumber("arm/voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("arm/cancoder radians", getMeasurement());
        SmartDashboard.putNumber("arm/velocity radians", getCANCoderVelocityRadians());
    }
}
