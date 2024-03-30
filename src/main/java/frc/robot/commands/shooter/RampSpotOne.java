package frc.robot.commands.shooter;

import frc.robot.Constants;

// No longer used
public class RampSpotOne extends RampShooter {

    public RampSpotOne() {
        super(
            Constants.ShootingConstants.spotOneRPM, 
            Constants.ShootingConstants.spotOneRPM, 
            Constants.ShootingConstants.spotOneAngle
        );
    }
}
