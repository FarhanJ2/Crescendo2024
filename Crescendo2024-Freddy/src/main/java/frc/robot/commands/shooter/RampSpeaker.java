package frc.robot.commands.shooter;

import frc.robot.Constants;

public class RampSpeaker extends RampShooter {

    public RampSpeaker() {
        super(Constants.ShootingConstants.speakerRPM, Constants.ShootingConstants.speakerRPM, Constants.ShootingConstants.speakerAngle);
    }
}
