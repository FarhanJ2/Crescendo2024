package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class HomeCommand extends Command {
    // private Shooter s_Shooter;

    public HomeCommand() {
        // this.s_Shooter = s_Shooter;
        addRequirements(RobotContainer.s_Shooter);
    }

    public void execute() {
        RobotContainer.s_Shooter.goHome();
        // s_Shooter.runShooterIdle();
    }

    public void end(boolean interrupted) {
        RobotContainer.s_Shooter.stopPivot();
    }

}
