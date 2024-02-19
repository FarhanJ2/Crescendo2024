package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.commands.AmpArm.ArmHandoff;
import frc.robot.commands.intake.ForkCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Intake;

public class AmpArm extends ProfiledPIDSubsystem {

    public static enum Position {
        HOME(Constants.AmpArm.homePosition),
        AMP(Constants.AmpArm.ampPosition),
        TRAP(Constants.AmpArm.trapPosition);

        private double rotations;

        private Position(double rotations) {
            this.rotations = rotations;
        }

        private double getRotations() {
            return rotations;
        }
    }

    private final TalonFX m_pivotMotor = new TalonFX(Constants.AmpArm.pivotMotorID);
    private final TalonFX m_shootMotor = new TalonFX(Constants.AmpArm.shootMotorID);

    // private final DigitalInput m_limitSwitch = new DigitalInput(Constants.AmpArm.limitSwitchChannel);

    // private final CANcoder m_cancoder = new CANcoder(Constants.AmpArm.canCoderID);

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
            0);

        getController().setTolerance(Constants.AmpArm.pivotTolerance);
        getController().setIZone(Constants.AmpArm.integratorZone);
        // setGoal(Constants.AmpArm.armOffset);

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

    public Command getAmpCommand() {
        return new InstantCommand(() -> setGoal(Position.AMP.getRotations()));
    }

    public Command getTrapCommand() {
        return new InstantCommand(() -> setGoal(Position.TRAP.getRotations()));
    }

    public Command getHomeCommand() {
        return new InstantCommand(() -> {
            setGoal(Position.HOME.getRotations());
            this.enable(); 
        });
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_pivotMotor.setVoltage(output + feedforward);
    }

    public void manualArmPivot(boolean pivotingUp) {
        double speed = Constants.AmpArm.manualArmPivotSpeed;
        if(!pivotingUp) {
            speed = -speed;
        }
        m_pivotMotor.set(speed);
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
        return getPivotRadians();
        // return m_pivotMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        
        //0.195 for w/o note; 0.22 for w/ note
        // m_pivotMotor.setVoltage(0.195); // stowing voltage TODO: make sure to include w/ vs w/o note
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
            // System.out.println(m_controller.getSetpoint().position);
        }

        // SmartDashboard.putNumber("testNum", 0);
        // System.out.println(SmartDashboard.getNumber("testNum", 0));
        
        SmartDashboard.putNumber("arm/rotations", m_pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("arm/degrees", getPivotDegrees());
        SmartDashboard.putNumber("arm/radians", getMeasurement());
    }
}
