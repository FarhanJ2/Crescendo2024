package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ManualShooterPivot extends Command {
  
  private Supplier<Boolean> pivotingUp;

  public ManualShooterPivot(Supplier<Boolean> pivotingUp) {
    this.pivotingUp = pivotingUp;
    addRequirements(RobotContainer.s_Shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_Shooter.manualShooterPivot(this.pivotingUp.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
