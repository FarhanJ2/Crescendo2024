package frc.lib.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DoubleClickJoystickButton extends JoystickButton {

    private final EventLoop m_loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    private final double debounce = 0.5; // in seconds
    private int counter;
    private BooleanSupplier m_condition;
    
    public DoubleClickJoystickButton(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
        m_condition = () -> joystick.getRawButton(buttonNumber);
    }

    /**
   * Starts the given command whenever the condition changes from `false` to `true` twice within 100 ms.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
    public Trigger onDoubleTap(Command command) {
        requireNonNullParam(command, "command", "onTrue");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();
                private double m_pressedLastTime = Timer.getFPGATimestamp();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();
                    double currentTime = Timer.getFPGATimestamp();

                    if (currentTime - m_pressedLastTime > debounce) {
                        counter = 0;
                    }
                    if (!m_pressedLast && pressed) {
                        m_pressedLastTime = Timer.getFPGATimestamp();
                        counter++;
                    }
                    if (counter == 2) {
                        counter = 0;
                        command.schedule();
                    }

                    m_pressedLast = pressed;
                }
            });
        return this;
    }
}