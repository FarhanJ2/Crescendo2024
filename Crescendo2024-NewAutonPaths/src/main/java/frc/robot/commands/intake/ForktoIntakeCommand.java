package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ForktoIntakeCommand extends Command {

    public ForktoIntakeCommand() {
        
        addRequirements(RobotContainer.s_Intake);
    }

    @Override
    public void execute() {
        RobotContainer.s_Intake.runForkToIntake();
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