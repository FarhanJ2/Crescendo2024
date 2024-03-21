package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ForkPreventArmCommand extends Command {
    
    public ForkPreventArmCommand() {        
        addRequirements(RobotContainer.s_Intake);
    }

    @Override
    public void execute() {
        RobotContainer.s_Intake.forkToShooterLow();
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
