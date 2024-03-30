package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RampAmp extends RampShooter {
    public RampAmp() {
        super(
            Constants.ShootingConstants.ampTopRPM, 
            Constants.ShootingConstants.ampBottomRPM, 
            Constants.ShootingConstants.ampAngle
        );
    }

    @Override
    public void initialize() {
        RobotContainer.s_Shooter.isScoringAmp = true;
    }
}
