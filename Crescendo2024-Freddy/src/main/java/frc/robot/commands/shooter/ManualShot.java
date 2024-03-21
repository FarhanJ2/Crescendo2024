// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ManualShot extends Command {

  public ManualShot() {
    addRequirements(RobotContainer.s_Shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_Shooter.manualShoot();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
