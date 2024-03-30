package frc.robot.commands.ampArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ArmHandoff extends Command {

  private boolean prevBroken;
  private boolean toArm;

  public ArmHandoff(boolean toArm) {
    prevBroken = false;
    this.toArm = toArm; // alternative is going back to intake
    addRequirements(RobotContainer.s_AmpArm);
  }

  @Override
  public void initialize() {
    prevBroken = false;
  }

  @Override
  public void execute() {
    if (toArm && RobotContainer.s_AmpArm.inHandoffPosition()) {
      RobotContainer.s_AmpArm.armHandoff();
    }
    
    if (RobotContainer.s_Intake.armBeamBroken()) {
      prevBroken = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_AmpArm.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return !RobotContainer.s_Intake.armBeamBroken() && prevBroken;
  }
}
