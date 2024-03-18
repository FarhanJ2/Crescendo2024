package frc.robot.commands.shooter;

import frc.robot.Constants;

public class RampCenterLine extends RampShooter {

    public RampCenterLine() {
        super(Constants.ShootingConstants.centerToAmpRPM, Constants.ShootingConstants.centerToAmpRPM, Constants.ShootingConstants.canterToAmpAngle);
    }
}
