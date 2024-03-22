package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDMode;

public class LedCommand extends Command {

  private LED.LEDColor color;
  private LED.LEDMode mode;
  private final double pulseInterval = 0.1;

  public LedCommand(LED.LEDColor color, LED.LEDMode mode) {
    this.color = color;
    this.mode = mode;

    // addRequirements(RobotContainer.s_Led);
  }

  public LedCommand(LED.LEDColor color) {
    this.color = color;
    this.mode = LEDMode.STATIC;

    // addRequirements(RobotContainer.s_Led);
  }

  @Override
  public void initialize() {
    if (mode != LED.LEDMode.RAINBOW) {
      // RobotContainer.s_Led.setColor(color);
    }
  }

  @Override
  public void execute() {

    if (mode == LED.LEDMode.PULSE) {
      // RobotContainer.s_Led.pulse(color, pulseInterval);
    }
    else if (mode == LED.LEDMode.STATIC) {
      // RobotContainer.s_Led.setColor(color);

    }
    else if (mode == LED.LEDMode.RAINBOW) {
      // RobotContainer.s_Led.rainbow();
    }
    else if (mode == LED.LEDMode.WAVE) {
      // RobotContainer.s_Led.wave(color);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (mode == LED.LEDMode.STATIC) {
      return true;
    }
    else {
      return false;
    }
  }
}