// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    // System.out.println(pivotingUp.get());
  
    if(this.pivotingUp.get() != null) {
      RobotContainer.s_AmpArm.manualArmPivot(this.pivotingUp.get());
    } else {
      RobotContainer.s_AmpArm.stopArm();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // RobotContainer.s_AmpArm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
