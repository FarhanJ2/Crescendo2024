// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ManualElevator extends Command {
  
  private Supplier<Boolean> isUp;

  public ManualElevator(Supplier<Boolean> isUp) {
    this.isUp = isUp;
    addRequirements(RobotContainer.s_Elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(this.isUp.get() != null) {
      RobotContainer.s_Elevator.moveElevator(this.isUp.get());  
    }
    else {
      RobotContainer.s_Elevator.stopElevator();
    }

  
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Elevator.stopElevator();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
