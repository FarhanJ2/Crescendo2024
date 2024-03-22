package frc.robot.commands.shooter;

import frc.robot.Constants;

public class RampPodium extends RampShooter {

    public RampPodium() {
        super(Constants.ShootingConstants.podiumRPM, Constants.ShootingConstants.podiumRPM, Constants.ShootingConstants.podiumAngle);
    }
}
