// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
