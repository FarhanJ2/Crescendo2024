package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AmpForkCommand extends Command {

    public AmpForkCommand() {
        addRequirements(RobotContainer.s_Intake);
    }

    @Override
    public void execute() {
        RobotContainer.s_Intake.runForkAmp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Intake.stop();
    }
}