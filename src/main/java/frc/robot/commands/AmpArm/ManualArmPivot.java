package frc.robot.commands.AmpArm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ManualArmPivot extends Command {
  
  private Supplier<Boolean> pivotingUp;

  public ManualArmPivot(Supplier<Boolean> pivotingUp) {
    this.pivotingUp = pivotingUp;
    addRequirements(RobotContainer.s_AmpArm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_AmpArm.manualArmPivot(this.pivotingUp.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
