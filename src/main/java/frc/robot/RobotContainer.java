package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.NoteVisualizer;
import frc.robot.commands.ampArm.ArmShot;
import frc.robot.commands.ampArm.ManualArmPivot;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.commands.shooter.HomeCommand;
import frc.robot.commands.shooter.ManualShooterPivot;
import frc.robot.commands.shooter.RampAmp;
import frc.robot.commands.shooter.RampFerryShot;
import frc.robot.commands.shooter.RampPodium;
import frc.robot.commands.shooter.RampSubwoofer;
import frc.robot.commands.swerve.RotateToAngle;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AmpArm.ArmStatus;
import frc.robot.subsystems.LED.LEDColor;

public class RobotContainer {

    private static enum OperatorMode {
        END_GAME_MODE,
        NORMAL_MODE,
    }

    private static enum OperatorLock {
        UNLOCKED,
        LOCKED,
    }

    public static Command autonomousCommand;
    public static DriverStation.Alliance alliance;
    public static boolean addVisionMeasurement = true;

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Driver */
    private final Trigger ferryShotButton = driver.leftBumper();
    private final Trigger intakeButton = driver.rightBumper();
    private final Trigger slowModeButton = driver.rightTrigger();
    private final Trigger zeroGyroButton = driver.b();
    private final Trigger toggleVisionMeasurement = driver.povUp();

    /* Operator */
    private final int manualShootAxis = 1;
    private final int manualArmAxis = 5;
    private final int manualElevatorAxis = 1;

    private OperatorMode operatorMode = OperatorMode.NORMAL_MODE;
    private OperatorLock armManual = OperatorLock.LOCKED;
    private OperatorLock shooterManual = OperatorLock.LOCKED;
    private OperatorLock elevatorManual = OperatorLock.LOCKED;

    private final Trigger isNormalMode = new Trigger(() -> operatorMode == OperatorMode.NORMAL_MODE);
    private final Trigger elevatorLocked = new Trigger(() -> elevatorManual == OperatorLock.LOCKED);

    /* Auton selector */
    private static final DigitalInput[] autonSelector = {
        new DigitalInput(10),
        new DigitalInput(11),
        new DigitalInput(12),
        new DigitalInput(13),
        new DigitalInput(18),
        new DigitalInput(19),
        new DigitalInput(20),
        new DigitalInput(21),
        new DigitalInput(22),
        new DigitalInput(23),
        new DigitalInput(24)
    };

    // Must match auton names in pathplanner
    public static final String[] autonNames = {
        "4 piece",
        "3 note center",
        "4 piece reverse",
        "2 note center",
        "red 5",
        "blue 1",
        "blue 2",
        "blue 3",
        "blue 4",
        "blue 5",
        "nothing"
    };

    // Must correspond with auton names
    private static final boolean[] useVisionInAuton = {
        false,      // 4 piece
        true,       // 3 note center
        false,      // 4 piece reverse
        true,       // 2 note center
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final AmpArm s_AmpArm = new AmpArm();
    public static final Intake s_Intake = new Intake();
    public static final Shooter s_Shooter = new Shooter();
    public static final Feeder s_Feeder = new Feeder();
    public static final Elevator s_Elevator = new Elevator();
    public static final LED s_Led = new LED(Constants.Led.port, Constants.Led.length);


    // Alliance getter thread
    private final Thread allianceGetter = new Thread(() -> {

        while(!DriverStation.isDSAttached()) {
            DriverStation.reportWarning("attaching DS...", false);
        }
        DriverStation.reportWarning("DS attached", false);

        try {
            Thread.sleep(1000);
        } catch (Exception e) {}
        
        alliance = DriverStation.getAlliance().get();
        DriverStation.silenceJoystickConnectionWarning(true);
        s_Swerve.poseEstimatorInitializer.start();

        s_Led.setDefaultCommand(
            s_Led.waveCommand(alliance == DriverStation.Alliance.Blue ? LEDColor.BLUE : LEDColor.RED)
        );
    });


    public RobotContainer() {

        allianceGetter.start();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> driver.getHID().getPOV() == 180,              // Robot centric (pov down)
                () -> driver.getHID().getLeftTriggerAxis() > 0.5,   // Aim at speaker (left trigger)
                () -> driver.getHID().getLeftBumper()               // Aim at amp for ferry shot (left bumper)
            )
        );

        s_Shooter.setDefaultCommand(
            new HomeCommand(s_Shooter)
        );

        autonomousCommand = new PathPlannerAuto(autonNames[getSelected()]);

        // Set up note visualizer
        NoteVisualizer.setRobotPoseSupplier(s_Swerve::getPose);

        // Configure all commands
        registerNamedCommands();
        configureAbsoluteButtonBindings();
        configureNormalModeButtonBindings();
        configureEndGameButtonBindings();
        configureTriggers();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("subwoofer shot", 
            new ParallelRaceGroup(
                new WaitCommand(2),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
                        new ParallelDeadlineGroup(
                            new WaitCommand(0.3),
                            s_Shooter.feedToShooter()
                        )
                    ),
                    new RampSubwoofer()
                )
            )
        );
        
        // Ramps and shoots when ready
        NamedCommands.registerCommand("anywhere shot",
            new ParallelRaceGroup(
                new WaitCommand(4),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
                        new ParallelDeadlineGroup(
                            new WaitCommand(0.3),
                            s_Shooter.feedToTrigShooter()
                        )
                    ),
                    new RotateToAngle(
                        () -> s_Swerve.calculateTurnAngle(alliance == DriverStation.Alliance.Blue ? Constants.BlueTeamPoses.blueSpeakerPose : Constants.RedTeamPoses.redSpeakerPose, s_Swerve.getHeading().getDegrees() + 180), 
                        () -> s_Swerve.getHeading().getDegrees()
                    ),
                    Commands.parallel(
                        Commands.run(
                            () -> s_Shooter.setPivot(RobotContainer.s_Swerve.odometryImpl.getPivotAngle(alliance))
                        ),
                        Commands.runEnd(
                            () -> s_Shooter.rampShooter(
                                s_Shooter.getTrigShotRPM(s_Swerve.odometryImpl.getDistanceToSpeaker()), 
                                s_Shooter.getTrigShotRPM(s_Swerve.odometryImpl.getDistanceToSpeaker())
                            ),
                            () -> s_Shooter.stopShooter()
                        ),
                        new Feed()
                    )
                )
            )
        );

        // Only ramps from anywhere and runs feeder
        NamedCommands.registerCommand("anywhere ramp",
            Commands.parallel(
                Commands.run(
                    () -> s_Shooter.setPivot(RobotContainer.s_Swerve.odometryImpl.getPivotAngle(alliance))
                ),
                Commands.runEnd(
                    () -> s_Shooter.rampShooter(
                        s_Shooter.getTrigShotRPM(s_Swerve.odometryImpl.getDistanceToSpeaker()), 
                        s_Shooter.getTrigShotRPM(s_Swerve.odometryImpl.getDistanceToSpeaker())
                    ),
                    () -> s_Shooter.stopShooter()
                ),
                new Feed()
            )
        );

        // Feed from anywhere, only works if feed is already running from ramping
        NamedCommands.registerCommand("anywhere feed",
            new ParallelRaceGroup(
                new WaitCommand(2),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.3),
                        s_Shooter.feedToTrigShooter()
                    )
                )
            )
        );

        NamedCommands.registerCommand("intake", new IntakeCommand());
    }

    // Configures the button bindings that don't change depending on end game/normal game mode
    private void configureAbsoluteButtonBindings() {
        zeroGyroButton.onTrue(
            new InstantCommand(
                () -> s_Swerve.zeroHeading()
            )
        );

        ferryShotButton.whileTrue(
            new RampFerryShot()
        );

        toggleVisionMeasurement.onTrue(
            new InstantCommand(() -> addVisionMeasurement = !addVisionMeasurement)
        );
        
        slowModeButton.onTrue(
            new InstantCommand(
                () -> s_Swerve.toggleMultiplier()
            )
        );

        intakeButton.whileTrue(
            new IntakeCommand()
        );

        // Switch between normal and end game mode, changing leds depending on mode
        operator.start()
        .and(operator.back())
            .onTrue(
                new InstantCommand(() -> {
                    if (operatorMode == OperatorMode.NORMAL_MODE) {
                        operatorMode = OperatorMode.END_GAME_MODE;
                        s_Led.getDefaultCommand().cancel();
                        s_Led.removeDefaultCommand();
                        s_Led.setDefaultCommand(
                            s_Led.fadeCommand(LEDColor.PURPLE)
                        );

                    } else {
                        operatorMode = OperatorMode.NORMAL_MODE;
                        s_Led.getDefaultCommand().cancel();
                        s_Led.removeDefaultCommand();
                        s_Led.setDefaultCommand(
                            s_Led.waveCommand(alliance == DriverStation.Alliance.Blue ? LEDColor.BLUE : LEDColor.RED)
                        );
                    }
                })
            );
    }

    private void configureTriggers() {
        // Vibrate operator when ready to shoot
        new Trigger(
            () -> s_Shooter.isReadyToShoot()
        )
        .and(operator.leftBumper() // One of the ramp buttons is held
            .or(operator.leftTrigger())
            .or(operator.rightTrigger())
            .or(operator.rightBumper())
            .or(operator.povRight())
            .or(driver.leftBumper())
        )
        .onTrue(
            Commands.runOnce(
                () -> {
                    s_Shooter.shooterReadyLEDCommand();
                    operator.getHID().setRumble(RumbleType.kBothRumble, 1);
                }
        )).onFalse(
            Commands.runOnce(
                () -> {
                    operator.getHID().setRumble(RumbleType.kBothRumble, 0);
                }
        ));

        // Flash led when intake is done
        new Trigger(
            () -> RobotContainer.s_Intake.intakeBeamBroken()
        ).onTrue(
            s_Intake.intakeLedCommand()
                .deadlineWith(Commands.run(
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 1);
                }
            ))
            .andThen(
                Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
            )
        );
    }

    // Operator bindings for normal mode
    private void configureNormalModeButtonBindings() {
        // Outtake
        operator.povDown()
            .and(isNormalMode)
                .whileTrue(
                    new ReverseIntakeCommand()
                );

        // Manual intake
        operator.povUp()
            .and(isNormalMode)
                .whileTrue(
                    Commands.runEnd(
                        () -> s_Intake.intake(),
                        () -> s_Intake.stop(),
                        s_Intake
                    )
                );
        
        // Arm handoff
        operator.y()
            .and(isNormalMode)
            .and(new Trigger(() -> s_AmpArm.status == ArmStatus.NOTHING))
                .onTrue( // Handoff
                    new SequentialCommandGroup(
                        new InstantCommand(() -> s_AmpArm.status = ArmStatus.INTAKING),
                        s_AmpArm.getHandoffCommand(),
                        new WaitUntilCommand(
                            () -> s_AmpArm.getController().atGoal()
                        ).raceWith(
                            new WaitCommand(1.5) // 1.5 second timeout in case it doesn't go to goal exactly
                        ),
                        s_AmpArm.feedToArm(),
                        Commands.runEnd(
                            () -> s_AmpArm.armHandoff(),
                            () -> s_AmpArm.stopShooter()
                        ),
                        new InstantCommand(() -> s_AmpArm.status = ArmStatus.NOTHING)
                    ).alongWith(s_Led.fadeCommand(LEDColor.YELLOW))
                );

        // Arm amp position
        operator.x()
            .and(isNormalMode)
            .and(new Trigger(() -> s_AmpArm.status == ArmStatus.NOTHING))
                .onTrue( // Amp position
                    s_AmpArm.getAmpShootCommand()
                        .alongWith(s_Elevator.getAmpCommand())
                        .alongWith(s_Led.flashCommand(LEDColor.YELLOW, 0.2, 2))
                );

        // Shoot note
        operator.b()
            .and(isNormalMode)
                .whileTrue(
                    new ArmShot().alongWith(s_Led.setColorCommand(LEDColor.WHITE))
                );

        // Put arm and elevator at home
        operator.a()
            .and(isNormalMode)
                .onTrue( // Home
                    Commands.sequence(
                        s_AmpArm.getAmpDangleCommand(),
                        new WaitCommand(0.3),
                        s_Elevator.getHomeCommand(),
                        new ConditionalCommand(s_AmpArm.getHandoffCommand(), new InstantCommand(), s_Elevator::limitPressed)
                    ).alongWith(new InstantCommand(() -> s_AmpArm.status = ArmStatus.NOTHING)));
        
        // Intake from shooter
        operator.povLeft()
            .and(isNormalMode)
                .whileTrue(
                    s_Shooter.feedToIntakeFromShooter()
                );

        // Ramp podium
        operator.leftTrigger()
            .and(isNormalMode)
                .whileTrue(
                    new RampPodium()
                );

        // Ramp subwoofer
        operator.leftBumper()
            .and(isNormalMode)
                .whileTrue(
                    new RampSubwoofer()
                );
        
        // Ramp anywhere
        operator.rightBumper().whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> s_Shooter.setPivot(RobotContainer.s_Swerve.odometryImpl.getPivotAngle(alliance))
                ),
                Commands.runEnd(
                    () -> s_Shooter.rampShooter(
                        s_Shooter.getTrigShotRPM(s_Swerve.odometryImpl.getDistanceToSpeaker()), 
                        s_Shooter.getTrigShotRPM(s_Swerve.odometryImpl.getDistanceToSpeaker())
                    ),
                    () -> s_Shooter.stopShooter()
                ),
                new Feed()
            )
        );

        // Shoot from podium or subwoofer, or ferry shoot
        operator.rightTrigger()
            .and(isNormalMode)
                .and(() -> !s_Shooter.isScoringAmp).and(operator.rightBumper().negate())
                    .whileTrue(
                        s_Shooter.feedToShooter()
                    )
                    .onTrue(
                        s_Led.flashCommand(LEDColor.WHITE, 0.15, 0.8)
                    );

        // Shoot from anywhere
        operator.rightTrigger()
            .and(isNormalMode)
                .and(operator.rightBumper())
                    .whileTrue(
                        s_Shooter.feedToTrigShooter()
                    )
                    .onTrue(
                        s_Led.flashCommand(LEDColor.WHITE, 0.15, 0.8)
                    );

        // Shoot into amp
        operator.rightTrigger()
            .and(isNormalMode)
                .and(() -> s_Shooter.isScoringAmp)
                    .whileTrue(
                        s_Shooter.feedToShooterAmp()
                    )
                    .onTrue(
                        s_Led.flashCommand(LEDColor.WHITE, 0.15, 0.8)
                    );
        
        // Amp shot from shooter (doesnt work anymore)
        (operator.povRight()
        .or(operator.povUpRight())
        .or(operator.povDownRight()))
            .and(isNormalMode)
                .whileTrue(
                    new RampAmp()
                );

        // Manual control of shooter
        operator.leftStick()
            .and(isNormalMode)
                .onTrue(
                    new InstantCommand(
                        () -> {
                            if(shooterManual == OperatorLock.LOCKED) {
                                shooterManual = OperatorLock.UNLOCKED;
                                s_Shooter.disable();
                                s_Shooter.getDefaultCommand().cancel();
                                s_Shooter.setDefaultCommand(
                                    new ManualShooterPivot(
                                        () -> {
                                            if(!isNormalMode.getAsBoolean() || Math.abs(operator.getRawAxis(manualShootAxis)) < Constants.Deadbands.shooterDeadband) {
                                                return null;
                                            } else {
                                                return Math.signum(-operator.getRawAxis(manualShootAxis)) > 0;
                                            }
                                        }
                                    )
                                );
                                
                            } else {
                                shooterManual = OperatorLock.LOCKED;
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

        // Manual control of arm
        operator.rightStick()
            .and(isNormalMode)
                .onTrue(
                    new InstantCommand(
                        () -> {
                            if (armManual == OperatorLock.LOCKED) {
                                armManual = OperatorLock.UNLOCKED;
                                s_AmpArm.disable();
                                s_AmpArm.setDefaultCommand(
                                    new ManualArmPivot(
                                        () -> {
                                            if(!isNormalMode.getAsBoolean() || Math.abs(operator.getRawAxis(manualArmAxis)) < Constants.Deadbands.armDeadband) {
                                                return null;
                                            } else {
                                                return Math.signum(-operator.getRawAxis(manualArmAxis)) > 0;
                                            }
                                        }
                                    )
                                );
                            } else {
                                armManual = OperatorLock.LOCKED;
                                s_AmpArm.enable();
                                s_AmpArm.getDefaultCommand().cancel();
                                s_AmpArm.removeDefaultCommand();
                            }
                        }
                    )
                );
    }

    // Operator bindings for end game mode
    private void configureEndGameButtonBindings() {

        // For trap, which we probably wont ever get :(
        // operator.leftTrigger() 
        //     .and(isNormalMode.negate())
        //         .and(elevatorLocked)
        //             .onTrue( // Trap Position
        //                 Commands.parallel(
        //                     s_Elevator.getTrapCommand(),
        //                     s_AmpArm.getTrapCommand()
        //                 )
        //             );

        // operator.rightTrigger()
        //     .and(isNormalMode.negate())
        //         .whileTrue(
        //             new ArmShot()
        //         );

        // Climb position
        operator.y()
            .and(isNormalMode.negate())
                .and(elevatorLocked)
                    .onTrue(
                        Commands.sequence(
                            s_AmpArm.getClimbPosition(),
                            new WaitCommand(0.5),
                            s_Elevator.getClimbCommand()
                        )
                    );
        
        // Home position (is not pressed while climbing)
        operator.a()
            .and(isNormalMode.negate())
                .and(elevatorLocked)
                    .onTrue(
                        Commands.parallel(
                            s_Elevator.getHomeCommand(),
                            s_AmpArm.getAmpDangleCommand()
                        )
                    );
        
        // Manual elevator control
        operator.leftStick()
            .and(isNormalMode.negate())
                .onTrue(
                    new InstantCommand(
                        () -> {
                            if(elevatorManual == OperatorLock.LOCKED) {
                                s_AmpArm.setGoal(Constants.AmpArm.danglePosition);
                                s_AmpArm.enable();
                                elevatorManual = OperatorLock.UNLOCKED;
                                s_Elevator.disable();
                                s_Elevator.setDefaultCommand(
                                    new ManualElevator(
                                        () -> {
                                            if(isNormalMode.getAsBoolean() || Math.abs(operator.getRawAxis(manualElevatorAxis)) < Constants.Deadbands.climbDeadband) {
                                                return null;
                                            } else {
                                                return Math.signum(operator.getRawAxis(manualElevatorAxis)) > 0;
                                            }
                                        }
                                    )
                                );
                            } else {
                                elevatorManual = OperatorLock.LOCKED;
                                s_Elevator.getDefaultCommand().cancel();
                                s_Elevator.removeDefaultCommand();
                            }
                        }
                    )
                );
    }

    public static int getSelected() {
        for (int i = 0; i < autonSelector.length; i++) {
            if (autonSelector[i].get() == false) {
                return i;
            }
        }

        return 0;
    }

    public static boolean useVisionInAuton() {
        return useVisionInAuton[getSelected()];
    }

    public static Command getAutonomousCommand() {
        return autonomousCommand;
    }
}