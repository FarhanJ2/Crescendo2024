// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
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
    if (toArm) {
      RobotContainer.s_AmpArm.armHandoff();
    }
    // RobotContainer.s_AmpArm.armHandoff();
    
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
