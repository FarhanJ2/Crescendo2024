package frc.robot.commands.shooter;

import frc.robot.Constants;

public class RampFerryShot extends RampShooter {

    public RampFerryShot() {
        super(
            Constants.ShootingConstants.ferryShotRPM, 
            Constants.ShootingConstants.ferryShotRPM, 
            Constants.ShootingConstants.ferryShotAngle
        );
    }
}
