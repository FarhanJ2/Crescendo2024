package frc.robot.commands.shooter;

import frc.robot.Constants;

public class RampSubwoofer extends RampShooter {

    public RampSubwoofer() {
        super(
            Constants.ShootingConstants.subwooferRPM, 
            Constants.ShootingConstants.subwooferRPM, 
            Constants.ShootingConstants.subwooferAngle
        );
    }
}
