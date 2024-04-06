package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED.LEDColor;

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public enum State {
    TELEOP,
    AUTON
  }

  public static State state = State.AUTON;

  @Override
  public void driverStationConnected() {
    RobotContainer.alliance = DriverStation.getAlliance().get();
    
    DriverStation.silenceJoystickConnectionWarning(true);
        
    // s_Swerve.poseEstimatorInitializer.start();
    RobotContainer.s_Swerve.initializePoseEstimator();

    RobotContainer.s_Led.setDefaultCommand(
      RobotContainer.s_Led.setColorCommand(RobotContainer.alliance == DriverStation.Alliance.Blue ? LEDColor.BLUE : LEDColor.RED)
    );
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // addPeriodic(() -> RobotContainer.s_Swerve.updateOdometry(), Constants.visionUpdateRate);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putString("auton/auton", RobotContainer.getAutonName());
    
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    RobotContainer.s_Led.rainbow();
  }

  @Override
  public void autonomousInit() {
    state = State.AUTON;
    m_autonomousCommand = RobotContainer.autonomousCommand;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    state = State.TELEOP;
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
