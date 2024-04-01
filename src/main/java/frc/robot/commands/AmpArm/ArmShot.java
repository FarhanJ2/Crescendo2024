package frc.robot.commands.AmpArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ArmShot extends Command {

  public ArmShot() {
    addRequirements(RobotContainer.s_AmpArm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_AmpArm.shoot();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_AmpArm.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
