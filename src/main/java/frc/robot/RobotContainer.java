package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.NoteVisualizer;
import frc.robot.commands.AmpArm.ArmHandoff;
import frc.robot.commands.AmpArm.ArmShot;
import frc.robot.commands.AmpArm.ManualArmPivot;
import frc.robot.commands.Elevator.ManualElevator;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.intake.ForkCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.commands.led.LedCommand;
import frc.robot.commands.shooter.HomeCommand;
import frc.robot.commands.shooter.ManualShooterPivot;
import frc.robot.commands.shooter.ManualShot;
import frc.robot.commands.shooter.Pivot;
import frc.robot.commands.shooter.RampAmp;
import frc.robot.commands.shooter.RampCenterLine;
import frc.robot.commands.shooter.RampPodium;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.RampSpeaker;
import frc.robot.commands.shooter.RampSpotOne;
import frc.robot.commands.shooter.RampStartLine;
import frc.robot.commands.swerve.RotateToAngle;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.LEDColor;
import frc.robot.subsystems.LED.LEDMode;

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
        // while(!DriverStation.waitForDsConnection(0)) {
        //     DriverStation.reportWarning("SHET UP", false);
        // }
        //TODO fix for comp
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
    private final CommandXboxController operator = new CommandXboxController(1);
    private final Joystick sysIDJoystick = new Joystick(2);

    /* Drive Controls */
    private final Trigger robotCentricButton = driver.leftBumper();
    private final Trigger intakeButton = driver.rightBumper();
    private final Trigger slowModeButton = driver.rightTrigger();
    private final Trigger alignSpeakerButton = driver.leftTrigger();
    private final Trigger zeroGyroButton = driver.b();
    // private final Trigger pivotToAngleButton = driver.a(); 
    private final Trigger toggleVisionMeasurement = driver.povUp();
    private final Trigger centerLineShotButton = driver.povDown();

    public static boolean addVisionMeasurement = true;

    /* Operator Buttons */
    private final int manualShootAxis = 1;
    private final int manualArmAxis = 5;
    private final int manualElevatorAxis = 1;

    private final Trigger isNormalMode = new Trigger(() -> operatorMode == OperatorMode.NORMAL_MODE);
    private final Trigger elevatorLocked = new Trigger(() -> elevatorManual == OperatorLock.LOCKED);

    /* Auton selector */
    private static final DigitalInput[] autonSelector = {
        new DigitalInput(10), // red 1
        new DigitalInput(11), // red 2
        new DigitalInput(12), // red 3
        new DigitalInput(13), // red 4
        new DigitalInput(18), // red 5
        new DigitalInput(19), // blue 1
        new DigitalInput(20), // blue 2
        new DigitalInput(21), // blue 3
        new DigitalInput(22), // blue 4
        new DigitalInput(23), // blue 5
        new DigitalInput(24) // nothing
    };

    public static final String[] autonNames = {
        "red 1",
        "red 2",
        "red 3",
        "red 4",
        "",
        "",
        "",
        "",
        "red 5",
        "blue 1",
        "blue 2",
        "blue 3",
        "blue 4",
        "blue 5",
        "nothing"
    };

    // These commands must correspond to its selection on the selector
    private final Command[] autons = {
        new InstantCommand(),
        new PathPlannerAuto("2 piece"),
        new PathPlannerAuto("3 piece"),
        new PathPlannerAuto("Copy of 4 piece"),
        new PathPlannerAuto("Center line")
    };

    /* Sysid Tuning Controller */
    private final JoystickButton sysidX = new JoystickButton(sysIDJoystick, 1);
    private final JoystickButton sysidA = new JoystickButton(sysIDJoystick, 2);
    private final JoystickButton sysidB = new JoystickButton(sysIDJoystick, 3);
    private final JoystickButton sysidY = new JoystickButton(sysIDJoystick, 4);
    private final JoystickButton sysidLeftBumper = new JoystickButton(sysIDJoystick, 5);
    private final JoystickButton sysidRightBumper = new JoystickButton(sysIDJoystick, 6);
    private final JoystickButton sysidLeftTrigger = new JoystickButton(sysIDJoystick, 7);
    private final JoystickButton sysidLeftStick = new JoystickButton(sysIDJoystick, 11);
    private final JoystickButton sysidRightStick = new JoystickButton(sysIDJoystick, 12);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final AmpArm s_AmpArm = new AmpArm();
    public static final Intake s_Intake = new Intake();
    public static final Shooter s_Shooter = new Shooter();
    public static final Feeder s_Feeder = new Feeder();
    public static final Elevator s_Elevator = new Elevator();
    public static final LED s_Led = new LED(Constants.Led.port, Constants.Led.length);
    
    public RobotContainer() {

        allianceGetter.start();

        // DriverStation.silenceJoystickConnectionWarning(true);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                robotCentricButton,
                () -> alignSpeakerButton.getAsBoolean()
            )
        );

        s_Shooter.setDefaultCommand(
            new HomeCommand(s_Shooter)
        );
        // s_Shooter.setDefaultCommand(
        //     new ConditionalCommand(
        //         new RampShooter(500, 500, 1.05),
        //         new HomeCommand(
        //             s_Shooter
        //         ), 
        //         () -> Robot.state == Robot.State.AUTON
        //     )
        // );

        // s_AmpArm.setDefaultCommand(
        //     //s_AmpArm.getAmpCommand()
        //     s_AmpArm.getHomeCommand()
        // );

        s_Led.setDefaultCommand(
            s_Led.waveCommand(alliance == DriverStation.Alliance.Blue ? LEDColor.BLUE : LEDColor.RED)
        );

        // NamedCommands.registerCommand("ramp", new RampShooter(1000, 1000, 0.13));
        // NamedCommands.registerCommand("subwoofer shot", 
        //     new ParallelDeadlineGroup(
        //         new WaitCommand(1.6),
        //         new RampPodium(),
        //         new SequentialCommandGroup(
        //             new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
        //             new InstantCommand(() -> System.out.println("READY TO SHOOT")),
        //             new ParallelDeadlineGroup(
        //                 new WaitCommand(0.3),
        //                 s_Shooter.feedToShooter()
        //             )
        //         )
        //     )
        // );
        NamedCommands.registerCommand("subwoofer shot", 
            new ParallelRaceGroup(
                new WaitCommand(2),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
                        new InstantCommand(() -> System.out.println("READY TO SHOOT")),
                        new ParallelDeadlineGroup(
                            new WaitCommand(0.3),
                            s_Shooter.feedToShooter()
                        )
                    ),
                    new RampSpeaker()
                )
            )
        );
        NamedCommands.registerCommand("spot 1 shot", 
            new ParallelRaceGroup(
                new WaitCommand(4),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
                        new InstantCommand(() -> System.out.println("READY TO SHOOT")),
                        new ParallelDeadlineGroup(
                            new WaitCommand(0.5),
                            s_Shooter.feedToShooter()
                        )
                    ),
                    new RotateToAngle(
                        () -> s_Swerve.calculateTurnAngle(alliance == DriverStation.Alliance.Blue ? Constants.BlueTeamPoses.blueSpeakerPose : Constants.RedTeamPoses.redSpeakerPose, s_Swerve.getHeading().getDegrees() + 180), 
                        () -> s_Swerve.getHeading().getDegrees()
                    ),
                    new RampSpotOne()
                )
            )
        );
        NamedCommands.registerCommand("intake", new IntakeCommand());

        // Set up note visualizer
        NoteVisualizer.setRobotPoseSupplier(s_Swerve::getPose);

        // Configure the button bindings
        configureAbsoluteButtonBindings();
        configureNormalModeButtonBindings();
        configureEndGameButtonBindings();
        configureSysIdButtonBindings();
        configureLEDBindings();
    }

    private void configureAbsoluteButtonBindings() {
        zeroGyroButton.onTrue(
            new InstantCommand(
                () -> s_Swerve.zeroHeading()
            )
        );

        centerLineShotButton.whileTrue(
            new ParallelCommandGroup(
                //aligns to the amp
                new RotateToAngle(
                    () -> s_Swerve.calculateTurnAngle(alliance == DriverStation.Alliance.Blue ? Constants.BlueTeamPoses.blueAmpPose : Constants.RedTeamPoses.redAmpPose, s_Swerve.getHeading().getDegrees() + 180), 
                    () -> s_Swerve.getHeading().getDegrees(),
                    () -> centerLineShotButton.getAsBoolean()
                ),
                new RampCenterLine(),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> s_Shooter.isReadyToShoot()),
                    s_Shooter.feedToShooter()
                )
            )
        );

        toggleVisionMeasurement.onTrue(
            new InstantCommand(() -> addVisionMeasurement = !addVisionMeasurement)
        );

        // pivotToAngleButton.whileTrue(
        //     Commands.parallel(
        //         Commands.run(
        //             () -> s_Shooter.setPivot(RobotContainer.s_Swerve.odometryImpl.getPivotAngle(alliance))
        //         ),
        //         Commands.runEnd(
        //             () -> s_Shooter.rampShooter(3000, 3000),
        //             () -> s_Shooter.stopShooter()
        //         )
        //     )
        // );
        
        slowModeButton.onTrue(
            new InstantCommand(
                () -> s_Swerve.toggleMultiplier()
            )
        );
        // TODO removed after rotate while moving
        // alignSpeakerButton.onTrue(
        //     new RotateToAngle(
        //         () -> s_Swerve.calculateTurnAngle(alliance == DriverStation.Alliance.Blue ? Constants.BlueTeamPoses.blueSpeakerPose : Constants.RedTeamPoses.redSpeakerPose, s_Swerve.getHeading().getDegrees() + 180), 
        //         () -> s_Swerve.getHeading().getDegrees(),
        //         () -> alignSpeakerButton.getAsBoolean()
        //     )
        // );

        // flashButton.onTrue(
        //     new InstantCommand(() -> {
        //             if (s_Swerve.getFlashedLimelight() != null) {
        //                 s_Swerve.getFlashedLimelight().flashLimelight();
        //             }
        //         } 
        //     )
        // );

        intakeButton.whileTrue(
            new IntakeCommand()
        );

        operator.start()
        .and(operator.back())
            .onTrue(
                new InstantCommand(() -> {
                    if (operatorMode == OperatorMode.NORMAL_MODE) {
                        // lockManualControls();
                        operatorMode = OperatorMode.END_GAME_MODE;

                    } else {
                        // lockManualControls();
                        operatorMode = OperatorMode.NORMAL_MODE;
                    }
                })
            );
    }

    private void lockManualControls() {
        // clearDefaultCommands();

        s_Shooter.setDefaultCommand(
            new HomeCommand(
                s_Shooter
            )
        );

        armManual = OperatorLock.LOCKED;
        shooterManual = OperatorLock.LOCKED;
        elevatorManual = OperatorLock.LOCKED;
    }

    private void clearDefaultCommands() {
        s_AmpArm.disable();
        s_AmpArm.getDefaultCommand().cancel();
        s_AmpArm.removeDefaultCommand();

        s_Shooter.disable();
        s_Shooter.getDefaultCommand().cancel();
        s_Shooter.removeDefaultCommand();

        s_Elevator.disable();
        s_Elevator.getDefaultCommand().cancel();
        s_Elevator.removeDefaultCommand();
    }

    private void configureLEDBindings() {

        //Ready To Shoot
        new Trigger(
            () -> s_Shooter.isReadyToShoot()
        )
        .and(operator.leftBumper()
            .or(operator.leftTrigger())
            .or(operator.rightTrigger())
            .or(operator.rightBumper())
            .or(operator.povRight())
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
            .andThen(new InstantCommand(() -> System.out.println("intake done")))
        );
    }

    private void configureSysIdButtonBindings() {
        // sysidY
        //     .whileTrue(
        //         s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //     );
            
        // sysidA
        //     .whileTrue(
        //         s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //     );
        
        // sysidB
        //     .whileTrue(
        //         s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //     );

        // sysidX
        //     .whileTrue(
        //         s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //     );

        sysidX.whileTrue(
            s_Elevator.applykG()
        );

        sysidA.whileTrue(
            s_Elevator.applykV()
        );

        // sysidX.whileTrue(
        //     s_AmpArm.applykS()
        // );

        // sysidY.whileTrue(
        //     s_AmpArm.applykG()
        // );

        // sysidB.whileTrue(
        //     s_AmpArm.applykV()
        // );
        // sysidA.onTrue(
        //     new InstantCommand(() -> {s_AmpArm.setGoal(0); s_AmpArm.enable();})
        // );

        // sysidY.onTrue(
        //     Commands.runOnce(() -> s_Shooter.setGoal(1))
        // );

        // sysidB.onTrue(
        //     Commands.runOnce(() -> s_Shooter.setGoal(0))
        // );
        // sysidX.whileTrue(
        //     s_Shooter.applykV()
        // );

        // sysidY
        //     .whileTrue(
        //         s_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //     );
            
        // sysidA
        //     .whileTrue(
        //         s_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //     );
        
        // sysidB
        //     .whileTrue(
        //         s_Shooter.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //     );

        // sysidX
        //     .whileTrue(
        //         s_Shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //     );

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

        // sysidY.onTrue(
        //     new InstantCommand(() -> {System.out.println("running"); s_Elevator.setGoal(1.5); s_Elevator.enable();})
        // );

        // sysidA.onTrue(
        //     s_Elevator.getHomeCommand()
        // );

        // sysidX.onTrue(
        //     new InstantCommand(() -> s_Elevator.disable())
        // );
        
        // sysidLeftBumper
        //     .whileTrue(
        //         s_AmpArm.applykS()
        //     );

        // sysidRightBumper
        //     .whileTrue(
        //         s_AmpArm.applykG()
        //     );
        
        // sysidLeftTrigger
        //     .whileTrue(
        //         s_AmpArm.applykV()
        //     );

        sysidLeftBumper
            .whileTrue(
                s_Shooter.applykS()
            );

        sysidRightBumper
            .whileTrue(
                s_Shooter.applykG()
            );
        
        sysidLeftTrigger
            .whileTrue(
                s_Shooter.applykV()
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
        operator.leftTrigger() 
            .and(isNormalMode.negate())
                .and(elevatorLocked)
                    .onTrue( // Trap Position
                        Commands.parallel(
                            s_Elevator.getTrapCommand(),
                            s_AmpArm.getTrapCommand()
                        )
                    );

        operator.rightTrigger()
            .and(isNormalMode.negate())
                .whileTrue(
                    new ArmShot()
                );

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
        
        operator.a()
            .and(isNormalMode.negate())
                .and(elevatorLocked)
                    .onTrue(
                        Commands.parallel(
                            s_Elevator.getHomeCommand(),
                            s_AmpArm.getAmpDangleCommand() // TODO used to be home
                        )
                    );

        operator.leftStick()
            .and(isNormalMode.negate())
                .onTrue(
                    new InstantCommand(
                        () -> {
                            if(elevatorManual == OperatorLock.LOCKED) {
                                s_AmpArm.setGoal(Constants.AmpArm.danglePosition);
                                s_AmpArm.enable();
                                // s_AmpArm.getAmpDangleCommand().execute();
                                elevatorManual = OperatorLock.UNLOCKED;
                                s_Elevator.disable();
                                s_Elevator.setDefaultCommand(
                                    new ManualElevator(
                                        () -> {
                                            if(isNormalMode.getAsBoolean() || Math.abs(operator.getRawAxis(manualElevatorAxis)) < Constants.stickDeadband * Math.PI * Math.E) { //deadband for joystick
                                                return null;
                                            } else {
                                                return Math.signum(operator.getRawAxis(manualElevatorAxis)) > 0;
                                            }
                                        }
                                    )
                                );
                            } else {
                                elevatorManual = OperatorLock.LOCKED;
                                // s_Elevator.enable();
                                s_Elevator.getDefaultCommand().cancel();
                                s_Elevator.removeDefaultCommand();
                            }
                        }
                    )
                );
    }

    private void configureNormalModeButtonBindings() {
        // Normal Mode
        operator.povDown()
            .and(isNormalMode)
                .whileTrue(
                    new ReverseIntakeCommand()
                ); // Outtake
        operator.povUp()
            .and(isNormalMode)
                .whileTrue(
                    Commands.runEnd( // Manual intake
                        () -> s_Intake.intake(),
                        () -> s_Intake.stop(),
                        s_Intake
                    )
                );
        // Arm
        operator.y()
            .and(isNormalMode)
                .onTrue( // Handoff
                    new SequentialCommandGroup(
                        s_AmpArm.getHandoffCommand(),
                        new WaitUntilCommand(
                            () -> s_AmpArm.getController().atGoal()
                        ).raceWith(
                            new WaitCommand(1) // 2 second timeout in case doesn't go to goal
                        ),
                        s_AmpArm.feedToArm(),
                        Commands.runEnd(
                            () -> s_AmpArm.armHandoff(),
                            () -> s_AmpArm.stopShooter()
                        ).withTimeout(0.1)
                    ).alongWith(s_Led.fadeCommand(LEDColor.YELLOW))
                );
        operator.x()
            .and(isNormalMode)
                .onTrue( // Amp position
                    s_AmpArm.getAmpShootCommand()
                        .alongWith(s_Elevator.getAmpCommand())
                        .alongWith(s_Led.flashCommand(LEDColor.YELLOW, 0.2, 2))
                );

        operator.b()
            .and(isNormalMode)
                .whileTrue(
                    new ArmShot().alongWith(s_Led.setColorCommand(LEDColor.WHITE))
                );
                // .onTrue(
                //     s_AmpArm.getAmpShootCommand()
                // )
                // .whileTrue( // Shoot
                //     new ArmShot().onlyIf(() -> s_AmpArm.getController().atGoal())
                // );
        operator.a()
            .and(isNormalMode)
                .onTrue( // Home
                    s_AmpArm.getAmpDangleCommand() // used to be home
                        .alongWith(s_Elevator.getHomeCommand())
                );
        // operator.povLeft()
        //     .and(isNormalMode)
        //         .onTrue( // Arm to intake
        //             new SequentialCommandGroup(
        //                 s_AmpArm.getHandoffCommand(),
        //                 new WaitUntilCommand(
        //                     () -> s_AmpArm.getController().atGoal()
        //                 ),
        //                 s_AmpArm.armToIntake()
        //             )
        //         );
        operator.povLeft()
            .and(isNormalMode)
                .whileTrue(
                    s_Shooter.feedToIntakeFromShooter()
                );

        // Shooter
        operator.leftTrigger()
            .and(isNormalMode)
                .whileTrue( // Podium shot
                    new ParallelCommandGroup(
                        // new Feed(),
                        new RampPodium() // 0.15 angle 1800-2000rpm for podium
                    )
                );
        operator.leftBumper()
            .and(isNormalMode)
                .whileTrue( // Speaker shot
                    new RampSpeaker()
                );


        //shooting speaker and amp
        operator.rightTrigger()
            .and(isNormalMode)
                .and(() -> !s_Shooter.isScoringAmp).and(operator.rightBumper().negate())
                    .whileTrue(
                        s_Shooter.feedToShooter()
                    )
                    .onTrue(
                        s_Led.flashCommand(LEDColor.WHITE, 0.15, 1)
                    );
                    // .onFalse(
                    //     s_Led.stopCommand()
                    // );

        operator.rightTrigger()
            .and(isNormalMode)
                .and(operator.rightBumper())
                    .whileTrue(
                        s_Shooter.feedToTrigShooter()
                    )
                    .onTrue(
                        s_Led.flashCommand(LEDColor.WHITE, 0.15, 1)
                    )
                    .onFalse(
                        s_Led.stopCommand()
                    );


        operator.rightTrigger()
            .and(isNormalMode)
                .and(() -> s_Shooter.isScoringAmp)
                    .whileTrue(
                        s_Shooter.feedToShooterAmp()
                    )
                    .onTrue(
                        s_Led.flashCommand(LEDColor.WHITE, 0.15, 1)
                    )
                    .onFalse(
                        s_Led.stopCommand()
                    );
        
        // TODO SHOOT FROM ANYWHERE, PUT BACK
        // operator.rightBumper()
        // .and(isNormalMode)
        //         .whileTrue( // Auto shoot ---- shoot from anywhere
        //             s_Shooter.readyShootCommand(
        //                 () -> s_Swerve.odometryImpl.getDistance(
        //                     RobotContainer.alliance == DriverStation.Alliance.Blue ? 
        //                     Constants.BlueTeamPoses.blueSpeakerPose : 
        //                     Constants.RedTeamPoses.redSpeakerPose
        //                 )
        //             )
        //         );
        // operator.rightBumper()
        //     .and(isNormalMode)
        //         .whileTrue(
        //             new RampStartLine()
        //         );

        // operator.rightBumper().onTrue(
        //     new InstantCommand(() -> {
        //         s_Shooter.trigTargetAngle = RobotContainer.s_Swerve.odometryImpl.getPivotAngle(alliance);
        //         System.out.println(s_Shooter.trigTargetAngle);
        //     })
        // );

        operator.rightBumper().whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> s_Shooter.setPivot(RobotContainer.s_Swerve.odometryImpl.getPivotAngle(alliance))
                ),
                Commands.runEnd(
                    () -> s_Shooter.rampShooter(3000, 3000),
                    () -> s_Shooter.stopShooter()
                ),
                new Feed()
            )
        );
        
        operator.povRight()
            .and(isNormalMode)
                .whileTrue(
                    new RampAmp()
                );

        // Locks
        // TODO don't change default commands, just disable the controller
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
                                            if(!isNormalMode.getAsBoolean() || Math.abs(operator.getRawAxis(manualShootAxis)) < Constants.stickDeadband) { //deadband for joystick
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


        operator.rightStick()
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
                                            if(!isNormalMode.getAsBoolean() || Math.abs(operator.getRawAxis(manualArmAxis)) < Constants.stickDeadband) { //deadband for joystick
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
                                s_AmpArm.enable();
                                s_AmpArm.getDefaultCommand().cancel();
                                s_AmpArm.removeDefaultCommand();
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

    public Command getAutonomousCommand() {
        // Copy of 4 piece
        // return new PathPlannerAuto("Copy of 4 piece");
        // return new PathPlannerAuto("Center line revised");
        // return new PathPlannerAuto("2 note center");
        return new PathPlannerAuto("3 note center");

        // return autons[getSelected()];
    }
}