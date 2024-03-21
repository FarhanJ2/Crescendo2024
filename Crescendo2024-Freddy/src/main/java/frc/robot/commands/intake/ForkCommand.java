package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ForkCommand extends Command {

    private Intake.Direction direction;
    
    public ForkCommand(Intake.Direction direction) {
        this.direction = direction;
        
        addRequirements(RobotContainer.s_Intake);
    }

    @Override
    public void execute() {
        
        RobotContainer.s_Intake.runFork(this.direction);
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
