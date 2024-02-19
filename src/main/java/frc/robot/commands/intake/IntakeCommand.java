package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {

    private boolean beamStartBroken;
    
    public IntakeCommand() {
        addRequirements(RobotContainer.s_Intake);
    }

    @Override
    public void initialize() {
        beamStartBroken = RobotContainer.s_Intake.intakeBeamBroken();
    }

    @Override
    public void execute() {
        RobotContainer.s_Intake.intake();
    }

    @Override
    public boolean isFinished() {
        if (beamStartBroken) {
            return false;
        }
        return RobotContainer.s_Intake.intakeBeamBroken();
        // return false;
        // return RobotContainer.s_Intake.beamBroken();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Intake.stop();
    }
}
