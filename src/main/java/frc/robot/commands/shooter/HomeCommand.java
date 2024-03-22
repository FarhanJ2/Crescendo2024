package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class HomeCommand extends Command {
    private Shooter s_Shooter;

    public HomeCommand(Shooter s_Shooter) {
        this.s_Shooter = s_Shooter;
        addRequirements(s_Shooter);
    }

    public void execute() {
        //TODO Test after getting CANCoder stuff
        s_Shooter.goHome();
        s_Shooter.rampShooter(Constants.ShootingConstants.cruiseRPM, Constants.ShootingConstants.cruiseRPM);
    }

    public void end(boolean interrupted) {
        s_Shooter.stopPivot();
    }

}
