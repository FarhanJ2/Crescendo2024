package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Feed extends Command {
    
    public Feed() {
        addRequirements(RobotContainer.s_Feeder);
    }

    @Override
    public void execute() {
        RobotContainer.s_Feeder.feed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Feeder.stopFeed();
    }
}
