package frc.robot;

import java.sql.Driver;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AmpArm.ArmHandoff;
import frc.robot.commands.AmpArm.ArmShot;
import frc.robot.commands.AmpArm.ManualArmPivot;
import frc.robot.commands.Elevator.ManualElevator;
import frc.robot.commands.intake.ForkCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.commands.led.LedCommand;
import frc.robot.commands.shooter.HomeCommand;
import frc.robot.commands.shooter.ManualShooterPivot;
import frc.robot.commands.shooter.ManualShot;
import frc.robot.commands.shooter.Pivot;
import frc.robot.commands.shooter.RampPodium;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.RampSpeaker;
import frc.robot.commands.swerve.RotateToAngle;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.LEDColor;
import frc.robot.subsystems.LED.LEDMode;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private static enum OperatorMode {
        END_GAME_MODE,
        NORMAL_MODE,
    }

    private static enum OperatorLock {
        UNLOCKED,
        LOCKED,
    }
   
    private final Thread allianceGetter = new Thread(() -> {
        while(!DriverStation.waitForDsConnection(0)) {
            DriverStation.reportWarning("SHET UP", false);
        }
        DriverStation.reportWarning("ME SHET UP???", false);
        try {
            Thread.sleep(1000);
        } catch (Exception e) {}
        
        alliance = DriverStation.getAlliance().get();
        DriverStation.silenceJoystickConnectionWarning(true);
        s_Swerve.poseEstimatorInitializer.start();
    });

    public static DriverStation.Alliance alliance;

    private OperatorMode operatorMode = OperatorMode.NORMAL_MODE;
    private OperatorLock armManual = OperatorLock.LOCKED;
    private OperatorLock shooterManual = OperatorLock.LOCKED;
    private OperatorLock elevatorManual = OperatorLock.LOCKED;

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick sysIDJoystick = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, 3); // B
    // private final JoystickButton robotCentric = new JoystickButton(driver, 5); // LB
    // private final JoystickButton slowMode = new JoystickButton(driver, 8); // RT
    // private final JoystickButton flashButton = new JoystickButton(driver, 1); // X

    //XBox Controller
    private final Trigger zeroGyro = driver.b(); // B
    private final Trigger robotCentric = driver.leftBumper(); // LB
    private final Trigger slowMode = driver.rightTrigger(); // RT
    private final Trigger flashButton = driver.x(); // X
    private final Trigger alignSpeakerButton = driver.leftTrigger(); // LT
    private final Trigger intakeButton = driver.rightBumper(); // RB


    // private final JoystickButton trackApriltag = new JoystickButton(driver, 1); // X
    // private final JoystickButton alignSpeakerButton = new JoystickButton(driver, 7); // LT
    // private final POVButton leftPOV = new POVButton(driver, 270);
    // private final POVButton rightPOV = new POVButton(driver, 90);
    // private final POVButton upPOV = new POVButton(driver, 0);
    // private final POVButton downPOV = new POVButton(driver, 180);

    /* Operator Buttons */
    private final int manualShootAxis = 1;
    private final int manualArmAxis = 3;
    private final int manualElevatorAxis = 1;

    private final JoystickButton xButton_op = new JoystickButton(operator, 1);
    private final JoystickButton aButton_op = new JoystickButton(operator, 2);
    private final JoystickButton bButton_op = new JoystickButton(operator, 3);
    private final JoystickButton yButton_op = new JoystickButton(operator, 4);
    private final JoystickButton leftBumper_op = new JoystickButton(operator, 5);
    private final JoystickButton rightBumper_op = new JoystickButton(operator, 6);
    private final JoystickButton leftTrigger_op = new JoystickButton(operator, 7);
    private final JoystickButton rightTrigger_op = new JoystickButton(operator, 8);
    private final JoystickButton backButton_op = new JoystickButton(operator, 9);
    private final JoystickButton startButton_op = new JoystickButton(operator, 10);
    private final JoystickButton leftThumbstick_op = new JoystickButton(operator, 11);
    private final JoystickButton rightThumbstick_op = new JoystickButton(operator, 12);

    private final POVButton upPOV_op = new POVButton(operator, 0);
    private final POVButton leftPOV_op = new POVButton(operator, 270);
    private final POVButton rightPOV_op = new POVButton(operator, 90);
    private final POVButton downPOV_op = new POVButton(operator, 180);


    private final JoystickButton sysidX = new JoystickButton(sysIDJoystick, 1);
    private final JoystickButton sysidA = new JoystickButton(sysIDJoystick, 2);
    private final JoystickButton sysidB = new JoystickButton(sysIDJoystick, 3);
    private final JoystickButton sysidY = new JoystickButton(sysIDJoystick, 4);
    private final JoystickButton sysidLeftBumper = new JoystickButton(sysIDJoystick, 5);
    private final JoystickButton sysidRightBumper = new JoystickButton(sysIDJoystick, 6);
    private final JoystickButton sysidLeftTrigger = new JoystickButton(sysIDJoystick, 7);
    private final JoystickButton sysidLeftStick = new JoystickButton(sysIDJoystick, 11);
    private final JoystickButton sysidRightStick = new JoystickButton(sysIDJoystick, 12);



    
    private final Trigger isNormalMode = new Trigger(() -> operatorMode == OperatorMode.NORMAL_MODE);

    
    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final AmpArm s_AmpArm = new AmpArm();
    public static final Intake s_Intake = new Intake();
    public static final Shooter s_Shooter = new Shooter();
    public static final Feeder s_Feeder = new Feeder();
    public static final Elevator s_Elevator = new Elevator();
    public static final LED s_Led = new LED(Constants.Led.port, Constants.Led.length);
    

    public static NoteStatus noteStatus = NoteStatus.NONE;

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        allianceGetter.start();

        // DriverStation.silenceJoystickConnectionWarning(true);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                robotCentric
            )
        );

        s_Shooter.setDefaultCommand(
            new HomeCommand(
                s_Shooter
            )
        );

        // s_AmpArm.setDefaultCommand(
        //     //s_AmpArm.getAmpCommand()
        //     s_AmpArm.getHomeCommand()
        // );

        //TODO CHECK IF IT WORKS
        s_Led.setDefaultCommand(
            s_Led.waveCommand(alliance == DriverStation.Alliance.Blue ? LEDColor.BLUE : LEDColor.RED)
        );

        // NamedCommands.registerCommand("ramp", new RampShooter(1000, 1000, 0.13));
        //TODO CHANGE BASED ON WHERE WE'RE SHOOTING
        NamedCommands.registerCommand("shoot", 
            new ParallelDeadlineGroup(
                new WaitCommand(1.6),
                new RampSpeaker(),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
                    new InstantCommand(() -> System.out.println("READY TO SHOOT")),
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),   
                        s_Shooter.feedToShooter()
                    )
                )
            )
        );
        NamedCommands.registerCommand("intake", new IntakeCommand());

        // Configure the button bindings
        configureAbsoluteButtonBindings();
        configureNormalModeButtonBindings();
        configureEndGameButtonBindings();
        configureSysIdButtonBindings();
        configureLEDBindings();
    }

    private void configureAbsoluteButtonBindings() {
        zeroGyro.onTrue(
            new InstantCommand(
                () -> s_Swerve.zeroHeading()
            )
        );
        
        slowMode.onTrue(
            new InstantCommand(
                () -> s_Swerve.toggleMultiplier()
            )
        );
        // TODO need to change team for this
        alignSpeakerButton.onTrue(
            new RotateToAngle(
                () -> s_Swerve.calculateTurnAngle(Constants.BlueTeamPoses.redSpeakerPose, s_Swerve.getHeading().getDegrees() + 180), 
                () -> s_Swerve.getHeading().getDegrees(),
                () -> alignSpeakerButton.getAsBoolean()
            )
        );

        flashButton.onTrue(
            new InstantCommand(() -> {
                    if (s_Swerve.getFlashedLimelight() != null) {
                        s_Swerve.getFlashedLimelight().flashLimelight();
                    }
                } 
            )
        );

        intakeButton.whileTrue(
            new IntakeCommand()
        );

        startButton_op
        .and(backButton_op)
            .onTrue(
                new InstantCommand(() -> {
                    if (operatorMode == OperatorMode.NORMAL_MODE) {
                        operatorMode = OperatorMode.END_GAME_MODE;
                        lockManualControls();

                    } else {
                        operatorMode = OperatorMode.NORMAL_MODE;
                        lockManualControls();
                    }
                })
            );

    }

    private void lockManualControls() {
        armManual = OperatorLock.LOCKED;
        shooterManual = OperatorLock.LOCKED;
        elevatorManual = OperatorLock.LOCKED;
    }

    private void configureLEDBindings() {

        //Ready To Shoot
        new Trigger(
            () -> s_Shooter.isReadyToShoot()
        ).onTrue(
            /*s_Shooter.shooterReadyLEDCommand()*/
            new InstantCommand(() -> System.out.println("n"))
        );

        new Trigger(
            () -> RobotContainer.s_Intake.intakeBeamBroken()
        ).onTrue(
            // Commands.sequence(
            //     Commands.run(() -> {
            //         driver.getHID().setRumble(RumbleType.kBothRumble, 0.7);
            //     }).alongWith(
            //         /*s_Intake.intakeLedCommand()*/
            //     ).withTimeout(1),
            //     new WaitCommand(1),
            //     Commands.run(() -> {
            //         driver.getHID().setRumble(RumbleType.kBothRumble, 0);
            //     })
            // )



            // new SequentialCommandGroup(
            //     new ParallelDeadlineGroup(
            //         new WaitCommand(0.5),
            //         new InstantCommand(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.7))
            //     ),
            //     new WaitCommand(0.5),
            //     new InstantCommand(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
            // )

            Commands.run(
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 0.7);
                }
            )
            .withTimeout(1)
            .andThen(
                Commands.runOnce(
                    () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0)
                )
            )
            .andThen(new InstantCommand(() -> System.out.println("finish vibrate")))


            // TODO do this after led works
            // s_Intake.intakeLedCommand()
            //     .deadlineWith(Commands.run(
            //     () -> {
            //         driver.getHID().setRumble(RumbleType.kBothRumble, 0.7);
            //     }
            // ))
            // .andThen(
            //     Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
            // )
            // .andThen(new InstantCommand(() -> System.out.println("intake done")))
        );
    }

    private void configureSysIdButtonBindings() {
        // sysidY
        //     .whileTrue(
        //         s_AmpArm.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //     );
            
        // sysidA
        //     .whileTrue(
        //         s_AmpArm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //     );
        
        // sysidB
        //     .whileTrue(
        //         s_AmpArm.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //     );

        // sysidX
        //     .whileTrue(
        //         s_AmpArm.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //     );



        // sysidY
        //     .whileTrue(
        //         s_Elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //     );
            
        // sysidA
        //     .whileTrue(
        //         s_Elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //     );
        
        // sysidB
        //     .whileTrue(
        //         s_Elevator.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //     );

        // sysidX
        //     .whileTrue(
        //         s_Elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //     );

        // sysidY.onTrue(
        //     s_Elevator.getTrapCommand()
        // );

        sysidY.onTrue(
            new InstantCommand(() -> {System.out.println("running"); s_Elevator.setGoal(1.5); s_Elevator.enable();})
        );
        // sysidY.onTrue(new InstantCommand(() -> System.out.println("running")));

        sysidA.onTrue(
            s_Elevator.getHomeCommand()
        );

        sysidX.onTrue(
            new InstantCommand(() -> s_Elevator.disable())
        );
        
        sysidLeftBumper
            .whileTrue(
                s_Elevator.applykS()
            );

        sysidRightBumper
            .whileTrue(
                s_AmpArm.applykG()
            );
        
        sysidLeftTrigger
            .whileTrue(
                s_AmpArm.applykV()
            );

        sysidLeftStick
            .whileTrue(
                Commands.runEnd(
                    () -> s_Elevator.moveElevator(true), 
                    () -> s_Elevator.stopElevator(), 
                    s_Elevator
                )
            );

        sysidRightStick
            .whileTrue(
                Commands.runEnd(
                    () -> s_Elevator.moveElevator(false), 
                    () -> s_Elevator.stopElevator(), 
                    s_Elevator
                )
            );
    }

    private void configureEndGameButtonBindings() {
        // Endgame Mode
        leftTrigger_op 
            .and(isNormalMode.negate())
                .onTrue( // Trap Position
                    s_AmpArm.getTrapCommand()
                );

        rightTrigger_op
            .and(isNormalMode.negate())
                .whileTrue(
                    new ArmShot()
                );

        yButton_op
            .and(isNormalMode.negate())
                .onTrue(
                    s_Elevator.getClimbCommand()
                );
        
        aButton_op
            .and(isNormalMode.negate())
                .onTrue(
                    s_Elevator.getHomeCommand()
                );

        leftThumbstick_op
            .and(isNormalMode.negate())
                .onTrue(
                    new InstantCommand(
                        () -> {
                            if(elevatorManual == OperatorLock.LOCKED) {
                                elevatorManual = OperatorLock.UNLOCKED;
                                s_Elevator.disable();
                                s_Elevator.setDefaultCommand(
                                    new ManualElevator(
                                        () -> {
                                            if(Math.abs(operator.getRawAxis(manualElevatorAxis)) < Constants.stickDeadband) { //deadband for joystick
                                                return null;
                                            } else {
                                                return Math.signum(operator.getRawAxis(manualElevatorAxis)) > 0;
                                            }
                                        }
                                    )
                                );
                            } else {
                                elevatorManual = OperatorLock.LOCKED;
                                s_Elevator.enable();
                                s_Elevator.getDefaultCommand().cancel();
                                s_Elevator.removeDefaultCommand();
                            }
                        }
                    )
                );
    }


    private void configureNormalModeButtonBindings() {
        // Normal Mode
        
        // Intake
        rightBumper_op
            .and(isNormalMode)
                .whileTrue(
                    new IntakeCommand()
                ); // Intake
        downPOV_op
            .and(isNormalMode)
                .whileTrue(
                    new ReverseIntakeCommand()
                ); // Outtake
        upPOV_op
            .and(isNormalMode)
                .whileTrue(
                    Commands.runEnd( // Manual intake
                        () -> s_Intake.intake(),
                        () -> s_Intake.stop(),
                        s_Intake
                    )
                );
        // // Arm
        yButton_op
            .and(isNormalMode)
                .onTrue( // Handoff
                    new SequentialCommandGroup(
                        s_AmpArm.getHandoffCommand(),
                        new WaitUntilCommand(
                            () -> s_AmpArm.getController().atGoal()
                        ),
                        s_AmpArm.feedToArm()
                    )
                );
        // yButton_op
        //     .and(isNormalMode)
        //         .whileTrue(
        //             s_AmpArm.feedToArm()
        //         );
        xButton_op
            .and(isNormalMode)
                .onTrue( // Amp position
                    s_AmpArm.getAmpCommand()
                );
        bButton_op
            .and(isNormalMode)
                .whileTrue( // Shoot
                    new ArmShot()
                );
        aButton_op
            .and(isNormalMode)
                .onTrue( // Home
                    s_AmpArm.getHomeCommand()
                );
        leftPOV_op
            .and(isNormalMode)
                .onTrue( // Arm to intake
                    new SequentialCommandGroup(
                        s_AmpArm.getHomeCommand(),
                        new WaitUntilCommand(
                            () -> s_AmpArm.getController().atGoal()
                        ),
                        s_AmpArm.armToIntake()
                    )
                );

        // Shooter
        leftTrigger_op
            .and(isNormalMode)
                .whileTrue( // Podium shot
                    new ParallelCommandGroup(
                        new RampPodium() // 0.15 angle 1800-2000rpm for podium
                    )
                );
        leftBumper_op
            .and(isNormalMode)
                .whileTrue( // Speaker shot
                    new RampSpeaker()
                );
        //  TODO Maybe make it so you press once to ramp up, then press again to shoot
        rightTrigger_op
            .and(isNormalMode)
                .whileTrue( // Auto shoot ---- shoot from anywhere
                    new ParallelCommandGroup(
                        s_Shooter.readyShootCommand(
                    // TODO need to change based on alliance
                            () -> s_Swerve.odometryImpl.getDistance(
                              RobotContainer.alliance == DriverStation.Alliance.Blue ? 
                                Constants.BlueTeamPoses.blueSpeakerPose : 
                                Constants.RedTeamPoses.redSpeakerPose
                            )
                        )
                    )
                );

        rightPOV_op
          .whileTrue(
              s_Shooter.feedToShooter()
          );

        // Locks
        // TODO don't change default commands, just disable the controller
        leftThumbstick_op
            .and(isNormalMode)
                .onTrue(
                    new InstantCommand(
                        () -> {
                            if(shooterManual == OperatorLock.LOCKED) {
                                shooterManual = OperatorLock.UNLOCKED;
                                // System.out.println("UNLOCKED");
                                s_Shooter.disable();
                                s_Shooter.getDefaultCommand().cancel();
                                s_Shooter.setDefaultCommand(
                                    new ManualShooterPivot(
                                        () -> {
                                            if(Math.abs(operator.getRawAxis(manualShootAxis)) < Constants.stickDeadband) { //deadband for joystick
                                                return null;
                                            } else {
                                                return Math.signum(-operator.getRawAxis(manualShootAxis)) > 0;
                                            }
                                        }
                                    )
                                );
                                
                            } else {
                                shooterManual = OperatorLock.LOCKED;
                                // System.out.println("LOCKED");
                                s_Shooter.enable();
                                s_Shooter.getDefaultCommand().cancel();
                                s_Shooter.setDefaultCommand(
                                    new HomeCommand(
                                        s_Shooter
                                    )
                                );
                            }
                        }
                    )
                );

        rightThumbstick_op
            .and(isNormalMode)
                .onTrue(
                    new InstantCommand(
                        () -> {
                            if (armManual == OperatorLock.LOCKED) {
                                armManual = OperatorLock.UNLOCKED;
                                // System.out.println("UNLOCKED");
                                s_AmpArm.disable();
                                s_AmpArm.setDefaultCommand(
                                    new ManualArmPivot(
                                        () -> {
                                            if(Math.abs(operator.getRawAxis(manualArmAxis)) < Constants.stickDeadband) { //deadband for joystick
                                                return null;
                                            } else {
                                                return Math.signum(-operator.getRawAxis(manualArmAxis)) > 0;
                                            }
                                        }
                                    )
                                );
                            } else {
                                armManual = OperatorLock.LOCKED;
                                // System.out.println("LOCKED");
                                // s_A%mpArm.enable();
                                s_AmpArm.removeDefaultCommand();
                            }
                        }
                    )
                );
    }

    
    
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Auto 1");
    }
}